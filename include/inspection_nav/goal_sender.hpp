#pragma once

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

#include <netinet/in.h>

#include <cstddef>
#include <string>

#include "inspection_nav/inspection_nav_common.hpp"

namespace inspection_nav {

class GoalSender {
 public:
  GoalSender(ros::NodeHandle& nh,
             const std::string& config_file,
             const std::string& goal_frame,
             double goal_timeout_sec,
             bool continue_on_failure,
             int route_id,
             bool enable_gimbal_action,
             const std::string& gimbal_udp_ip,
             int gimbal_udp_port,
             double gimbal_udp_timeout_sec,
             double gimbal_udp_retry_interval_sec);

  ~GoalSender();

  bool run();

 private:
  bool notifyGimbalAndWait(std::size_t index, std::size_t total);
  bool ensureGimbalSocket();
  bool sendGimbalRequest(const std::string& payload);
  bool waitForGimbalResponse(std::string* response);
  bool responseAllowsContinue(const std::string& response) const;
  std::string buildGimbalRequest(std::size_t index, std::size_t total) const;

  ros::NodeHandle nh_;
  std::string config_file_;
  std::string goal_frame_;
  double goal_timeout_sec_;
  bool continue_on_failure_;
  int route_id_;
  bool enable_gimbal_action_;
  std::string gimbal_udp_ip_;
  int gimbal_udp_port_;
  double gimbal_udp_timeout_sec_;
  double gimbal_udp_retry_interval_sec_;

  int gimbal_socket_;
  sockaddr_in gimbal_addr_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
};

}  // namespace inspection_nav
