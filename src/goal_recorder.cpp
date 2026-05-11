#include "inspection_nav/goal_recorder.hpp"

#include <cerrno>
#include <iostream>
#include <string>
#include <thread>

#include <sys/select.h>
#include <unistd.h>

namespace inspection_nav {

GoalRecorder::GoalRecorder(ros::NodeHandle& nh, const std::string& config_file, int route_id)
    : nh_(nh),
      config_file_(config_file),
      route_id_(route_id),
      has_odom_(false),
      running_(true) {
  clicked_sub_ = nh_.subscribe("/clicked_point", 20, &GoalRecorder::clickedPointCb, this);
  odom_sub_ = nh_.subscribe("/odom", 50, &GoalRecorder::odomCb, this);

  ROS_INFO("Goal recorder started. Route: %d, config file: %s", route_id_, config_file_.c_str());
  ROS_INFO("Input methods:");
  ROS_INFO("1) Use RViz Publish Point to send /clicked_point goals");
  ROS_INFO("2) Press Enter in this terminal to record current /odom pose");
}

void GoalRecorder::run() {
  std::thread input_thread(&GoalRecorder::keyboardLoop, this);
  input_thread.join();
}

void GoalRecorder::odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(odom_mtx_);
  latest_odom_ = *msg;
  has_odom_ = true;
}

void GoalRecorder::clickedPointCb(const geometry_msgs::PointStamped::ConstPtr& msg) {
  NavPoint p;
  p.x = msg->point.x;
  p.y = msg->point.y;
  p.z = msg->point.z;

  if (!getLatestOrientation(&p.qx, &p.qy, &p.qz, &p.qw)) {
    p.qx = 0.0;
    p.qy = 0.0;
    p.qz = 0.0;
    p.qw = 1.0;
  }

  p.gimbal_inspect = 0;

  if (writePoint(p)) {
    ROS_INFO(
        "%s[GOAL RECORDED] Route: %d, Source: /clicked_point, point: (%.3f, %.3f, %.3f)%s",
        kColorGreen, route_id_, p.x, p.y, p.z, kColorReset);
  }
}

void GoalRecorder::keyboardLoop() {
  std::string input;
  while (ros::ok() && running_.load()) {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200000;

    const int ret = ::select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv);
    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }
      ROS_WARN_THROTTLE(5.0, "[RECORDER] stdin select failed, errno=%d", errno);
      continue;
    }

    if (ret == 0) {
      continue;
    }

    if (!std::getline(std::cin, input)) {
      if (!ros::ok()) {
        break;
      }
      std::cin.clear();
      continue;
    }

    if (!ros::ok()) {
      break;
    }

    NavPoint p;
    if (!getLatestOdomPoint(&p)) {
      ROS_WARN("[RECORD FAILED] Route: %d, /odom has not been received yet", route_id_);
      continue;
    }

    p.gimbal_inspect = 1;

    if (writePoint(p)) {
      ROS_INFO(
          "%s[GOAL RECORDED] Route: %d, Source: Enter key current pose, point: (%.3f, %.3f, %.3f)%s",
          kColorGreen, route_id_, p.x, p.y, p.z, kColorReset);
    }
  }
  running_.store(false);
}

bool GoalRecorder::getLatestOrientation(double* qx, double* qy, double* qz, double* qw) {
  if (qx == nullptr || qy == nullptr || qz == nullptr || qw == nullptr) {
    return false;
  }

  std::lock_guard<std::mutex> lock(odom_mtx_);
  if (!has_odom_) {
    return false;
  }

  *qx = latest_odom_.pose.pose.orientation.x;
  *qy = latest_odom_.pose.pose.orientation.y;
  *qz = latest_odom_.pose.pose.orientation.z;
  *qw = latest_odom_.pose.pose.orientation.w;
  return true;
}

bool GoalRecorder::getLatestOdomPoint(NavPoint* p) {
  if (p == nullptr) {
    return false;
  }

  std::lock_guard<std::mutex> lock(odom_mtx_);
  if (!has_odom_) {
    return false;
  }

  p->x = latest_odom_.pose.pose.position.x;
  p->y = latest_odom_.pose.pose.position.y;
  p->z = latest_odom_.pose.pose.position.z;
  p->qx = latest_odom_.pose.pose.orientation.x;
  p->qy = latest_odom_.pose.pose.orientation.y;
  p->qz = latest_odom_.pose.pose.orientation.z;
  p->qw = latest_odom_.pose.pose.orientation.w;
  return true;
}

bool GoalRecorder::writePoint(const NavPoint& p) {
  std::lock_guard<std::mutex> lock(file_mtx_);
  return GoalFileIO::appendPoint(config_file_, p);
}

}  // namespace inspection_nav
