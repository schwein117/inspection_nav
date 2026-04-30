#pragma once

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

#include <string>

#include "inspection_nav/inspection_nav_common.hpp"

namespace inspection_nav
{

  class GoalSender
  {
  public:
    GoalSender(ros::NodeHandle &nh,
               const std::string &config_file,
               const std::string &goal_frame,
               double goal_timeout_sec,
               bool continue_on_failure,
               int route_id,
               bool enable_gimbal_action);

    bool run();

  private:
    bool notifyGimbalAndWait(const NavPoint &p, std::size_t index, std::size_t total);

    ros::NodeHandle nh_;
    std::string config_file_;
    std::string goal_frame_;
    double goal_timeout_sec_;
    bool continue_on_failure_;
    int route_id_;
    bool enable_gimbal_action_;
    std::string gimbal_service_name_;
    std::string gimbal_action_name_;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
    ros::ServiceClient gimbal_client_;
  };

} // namespace inspection_nav
