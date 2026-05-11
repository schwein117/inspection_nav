#include "inspection_nav/goal_sender.hpp"

#include <geometry_msgs/Pose.h>

#include <arpa/inet.h>
#include <cerrno>
#include <cctype>
#include <cstring>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <sstream>

namespace {

std::string trimString(const std::string& s) {
  const std::string whitespace = " \t\r\n";
  const std::size_t start = s.find_first_not_of(whitespace);
  if (start == std::string::npos) {
    return "";
  }
  const std::size_t end = s.find_last_not_of(whitespace);
  return s.substr(start, end - start + 1);
}

std::string toUpperCopy(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return s;
}

}  // namespace

namespace inspection_nav {

GoalSender::GoalSender(ros::NodeHandle& nh,
                       const std::string& config_file,
                       const std::string& goal_frame,
                       double goal_timeout_sec,
                       bool continue_on_failure,
                       int route_id,
                       bool enable_gimbal_action,
                       const std::string& gimbal_udp_ip,
                       int gimbal_udp_port,
                       double gimbal_udp_timeout_sec,
                       double gimbal_udp_retry_interval_sec)
    : nh_(nh),
      config_file_(config_file),
      goal_frame_(goal_frame),
      goal_timeout_sec_(goal_timeout_sec),
      continue_on_failure_(continue_on_failure),
      route_id_(route_id),
      enable_gimbal_action_(enable_gimbal_action),
      gimbal_udp_ip_(gimbal_udp_ip),
      gimbal_udp_port_(gimbal_udp_port),
      gimbal_udp_timeout_sec_(gimbal_udp_timeout_sec),
      gimbal_udp_retry_interval_sec_(gimbal_udp_retry_interval_sec),
      gimbal_socket_(-1),
      ac_("move_base", true) {
  if (gimbal_udp_ip_.empty()) {
    gimbal_udp_ip_ = "127.0.0.1";
  }
  if (gimbal_udp_timeout_sec_ <= 0.0) {
    gimbal_udp_timeout_sec_ = 1.0;
  }
  if (gimbal_udp_retry_interval_sec_ <= 0.0) {
    gimbal_udp_retry_interval_sec_ = 1.0;
  }
  if (enable_gimbal_action_ && (gimbal_udp_port_ <= 0 || gimbal_udp_port_ > 65535)) {
    ROS_ERROR("Invalid gimbal UDP port: %d", gimbal_udp_port_);
    enable_gimbal_action_ = false;
  }
}

GoalSender::~GoalSender() {
  if (gimbal_socket_ >= 0) {
    ::close(gimbal_socket_);
    gimbal_socket_ = -1;
  }
}

bool GoalSender::ensureGimbalSocket() {
  if (gimbal_socket_ >= 0) {
    return true;
  }

  if (gimbal_udp_port_ <= 0 || gimbal_udp_port_ > 65535) {
    ROS_ERROR("Invalid gimbal UDP port: %d", gimbal_udp_port_);
    return false;
  }

  gimbal_socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (gimbal_socket_ < 0) {
    ROS_ERROR("Failed to create gimbal UDP socket: %s", std::strerror(errno));
    return false;
  }

  std::memset(&gimbal_addr_, 0, sizeof(gimbal_addr_));
  gimbal_addr_.sin_family = AF_INET;
  gimbal_addr_.sin_port = htons(static_cast<uint16_t>(gimbal_udp_port_));
  if (::inet_pton(AF_INET, gimbal_udp_ip_.c_str(), &gimbal_addr_.sin_addr) != 1) {
    ROS_ERROR("Invalid gimbal UDP IP: %s", gimbal_udp_ip_.c_str());
    ::close(gimbal_socket_);
    gimbal_socket_ = -1;
    return false;
  }

  if (::connect(gimbal_socket_, reinterpret_cast<sockaddr*>(&gimbal_addr_),
                sizeof(gimbal_addr_)) != 0) {
    ROS_ERROR("Failed to connect gimbal UDP socket: %s", std::strerror(errno));
    ::close(gimbal_socket_);
    gimbal_socket_ = -1;
    return false;
  }

  struct timeval tv;
  tv.tv_sec = static_cast<int>(gimbal_udp_timeout_sec_);
  tv.tv_usec = static_cast<int>((gimbal_udp_timeout_sec_ - tv.tv_sec) * 1000000.0);
  if (::setsockopt(gimbal_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0) {
    ROS_WARN("Failed to set UDP receive timeout: %s", std::strerror(errno));
  }

  return true;
}

std::string GoalSender::buildGimbalRequest(std::size_t index,
                                           std::size_t total) const {
  std::ostringstream ss;
  ss << "INSPECT," << route_id_ << "," << (index + 1) << "," << total;
  return ss.str();
}

bool GoalSender::sendGimbalRequest(const std::string& payload) {
  const ssize_t sent =
      ::send(gimbal_socket_, payload.c_str(), static_cast<int>(payload.size()), 0);
  if (sent < 0) {
    ROS_WARN("Gimbal UDP send failed: %s", std::strerror(errno));
    return false;
  }
  return true;
}

bool GoalSender::waitForGimbalResponse(std::string* response) {
  if (response == nullptr) {
    return false;
  }

  char buffer[256];
  const ssize_t len = ::recv(gimbal_socket_, buffer, sizeof(buffer) - 1, 0);
  if (len < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return false;
    }
    ROS_WARN("Gimbal UDP receive failed: %s", std::strerror(errno));
    return false;
  }

  buffer[len] = '\0';
  *response = std::string(buffer);
  return true;
}

bool GoalSender::responseAllowsContinue(const std::string& response) const {
  const std::string trimmed = trimString(response);
  if (trimmed.empty()) {
    return false;
  }

  const std::string upper = toUpperCopy(trimmed);
  if (upper == "1" || upper == "OK" || upper == "CONTINUE" || upper == "CONTINUE=1" ||
      upper == "TRUE") {
    return true;
  }

  return false;
}

bool GoalSender::notifyGimbalAndWait(std::size_t index, std::size_t total) {
  const std::string payload = buildGimbalRequest(index, total);

  ROS_INFO("[GIMBAL][UDP] Requesting inspect at goal %zu/%zu -> %s:%d", index + 1, total,
           gimbal_udp_ip_.c_str(), gimbal_udp_port_);

  while (ros::ok()) {
    if (!ensureGimbalSocket()) {
      ROS_WARN_THROTTLE(5.0, "[GIMBAL][UDP] Socket not ready, retrying...");
      ros::Duration(gimbal_udp_retry_interval_sec_).sleep();
      continue;
    }

    if (!sendGimbalRequest(payload)) {
      ROS_WARN_THROTTLE(5.0, "[GIMBAL][UDP] Send failed, retrying...");
      ros::Duration(gimbal_udp_retry_interval_sec_).sleep();
      continue;
    }

    std::string response;
    if (!waitForGimbalResponse(&response)) {
      ROS_WARN_THROTTLE(5.0, "[GIMBAL][UDP] Waiting for continue command...");
      ros::Duration(gimbal_udp_retry_interval_sec_).sleep();
      continue;
    }

    if (responseAllowsContinue(response)) {
      ROS_INFO("[GIMBAL][UDP] Continue navigation.");
      return true;
    }

    ROS_INFO("[GIMBAL][UDP] Response indicates waiting: %s", trimString(response).c_str());
    ros::Duration(gimbal_udp_retry_interval_sec_).sleep();
  }

  return false;
}

bool GoalSender::run() {
  std::vector<NavPoint> points;
  if (!GoalFileIO::loadPoints(config_file_, &points)) {
    return false;
  }

  if (points.empty()) {
    ROS_WARN("No goals found for route %d in config file: %s", route_id_, config_file_.c_str());
    return false;
  }

  ROS_INFO("Route %d selected. Loaded %zu goals. Waiting for move_base action server...",
           route_id_, points.size());
  ac_.waitForServer();
  ROS_INFO("move_base connected. Sending route %d goals in sequence.", route_id_);

  for (std::size_t i = 0; i < points.size() && ros::ok(); ++i) {
    const NavPoint& p = points[i];

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = goal_frame_;
    goal.target_pose.pose.position.x = p.x;
    goal.target_pose.pose.position.y = p.y;
    goal.target_pose.pose.position.z = p.z;
    goal.target_pose.pose.orientation.x = p.qx;
    goal.target_pose.pose.orientation.y = p.qy;
    goal.target_pose.pose.orientation.z = p.qz;
    goal.target_pose.pose.orientation.w = p.qw;

    ROS_INFO("[SEND GOAL %zu/%zu][Route %d] pos=(%.3f, %.3f, %.3f)", i + 1, points.size(),
             route_id_, p.x, p.y, p.z);

    ac_.sendGoal(goal);

    bool finished = false;
    if (goal_timeout_sec_ > 0.0) {
      finished = ac_.waitForResult(ros::Duration(goal_timeout_sec_));
    } else {
      ac_.waitForResult();
      finished = true;
    }

    if (!finished) {
      ROS_ERROR("[GOAL %zu][Route %d] Timed out before reaching destination", i + 1, route_id_);
      ac_.cancelGoal();
      if (!continue_on_failure_) {
        return false;
      }
      continue;
    }

    const actionlib::SimpleClientGoalState state = ac_.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("%s[GOAL REACHED] Route %d, Goal %zu/%zu%s", kColorGreen, route_id_, i + 1,
               points.size(), kColorReset);

      if (p.gimbal_inspect != 0) {
        if (!enable_gimbal_action_) {
          ROS_WARN("[GIMBAL][UDP] gimbal_inspect=1 but enable_gimbal_action=false, skipping wait.");
        } else {
          if (!notifyGimbalAndWait(i, points.size())) {
            return false;
          }
        }
      }
    } else {
      ROS_ERROR("[GOAL FAILED] Route %d, Goal %zu/%zu, state: %s", route_id_, i + 1,
                points.size(), state.toString().c_str());
      if (!continue_on_failure_) {
        return false;
      }
    }
  }

  ROS_INFO("%s[NAVIGATION FINISHED] Route %d completed%s", kColorGreen, route_id_,
           kColorReset);
  return true;
}

}  // namespace inspection_nav
