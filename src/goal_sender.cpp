#include "inspection_nav/goal_sender.hpp"

#include <geometry_msgs/Pose.h>

#include "inspection_nav/GimbalAction.h"

namespace inspection_nav
{

  GoalSender::GoalSender(ros::NodeHandle &nh,
                         const std::string &config_file,
                         const std::string &goal_frame,
                         double goal_timeout_sec,
                         bool continue_on_failure,
                         int route_id,
                         bool enable_gimbal_action)
      : nh_(nh),
        config_file_(config_file),
        goal_frame_(goal_frame),
        goal_timeout_sec_(goal_timeout_sec),
        continue_on_failure_(continue_on_failure),
        route_id_(route_id),
        enable_gimbal_action_(enable_gimbal_action),
        gimbal_service_name_("/gimbal/execute_action"),
        gimbal_action_name_("inspect"),
        ac_("move_base", true)
  {
    nh_.param<std::string>("gimbal_service_name", gimbal_service_name_, gimbal_service_name_);
    nh_.param<std::string>("gimbal_action_name", gimbal_action_name_, gimbal_action_name_);
    gimbal_client_ = nh_.serviceClient<inspection_nav::GimbalAction>(gimbal_service_name_);
  }

  bool GoalSender::notifyGimbalAndWait(const NavPoint &p, std::size_t index, std::size_t total)
  {
    inspection_nav::GimbalAction srv;
    srv.request.header.stamp = ros::Time::now();
    srv.request.header.frame_id = goal_frame_;
    srv.request.route_id = route_id_;
    srv.request.goal_index = static_cast<int32_t>(index + 1);
    srv.request.goal_total = static_cast<int32_t>(total);
    srv.request.action_name = gimbal_action_name_;
    srv.request.target_pose.position.x = p.x;
    srv.request.target_pose.position.y = p.y;
    srv.request.target_pose.position.z = p.z;
    srv.request.target_pose.orientation.x = p.qx;
    srv.request.target_pose.orientation.y = p.qy;
    srv.request.target_pose.orientation.z = p.qz;
    srv.request.target_pose.orientation.w = p.qw;

    ROS_INFO("[GIMBAL] Requesting action '%s' at goal %zu/%zu", gimbal_action_name_.c_str(),
             index + 1, total);

    while (ros::ok())
    {
      if (!gimbal_client_.exists())
      {
        ROS_WARN_THROTTLE(5.0, "[GIMBAL] Waiting for service: %s",
                          gimbal_service_name_.c_str());
        gimbal_client_.waitForExistence(ros::Duration(1.0));
        continue;
      }

      if (!gimbal_client_.call(srv))
      {
        ROS_WARN_THROTTLE(5.0, "[GIMBAL] Service call failed, retrying...");
        ros::Duration(1.0).sleep();
        continue;
      }

      if (srv.response.continue_nav)
      {
        ROS_INFO("[GIMBAL] Action done, continue navigation.");
        return true;
      }

      ROS_INFO("[GIMBAL] Action not finished yet, waiting... message: %s",
               srv.response.message.c_str());
      ros::Duration(1.0).sleep();
    }

    return false;
  }

  bool GoalSender::run()
  {
    std::vector<NavPoint> points;
    if (!GoalFileIO::loadPoints(config_file_, &points))
    {
      return false;
    }

    if (points.empty())
    {
      ROS_WARN("No goals found for route %d in config file: %s", route_id_, config_file_.c_str());
      return false;
    }

    ROS_INFO("Route %d selected. Loaded %zu goals. Waiting for move_base action server...",
             route_id_, points.size());
    ac_.waitForServer();
    ROS_INFO("move_base connected. Sending route %d goals in sequence.", route_id_);

    for (std::size_t i = 0; i < points.size() && ros::ok(); ++i)
    {
      const NavPoint &p = points[i];

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
      if (goal_timeout_sec_ > 0.0)
      {
        finished = ac_.waitForResult(ros::Duration(goal_timeout_sec_));
      }
      else
      {
        ac_.waitForResult();
        finished = true;
      }

      if (!finished)
      {
        ROS_ERROR("[GOAL %zu][Route %d] Timed out before reaching destination", i + 1, route_id_);
        ac_.cancelGoal();
        if (!continue_on_failure_)
        {
          return false;
        }
        continue;
      }

      const actionlib::SimpleClientGoalState state = ac_.getState();
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("%s[GOAL REACHED] Route %d, Goal %zu/%zu%s", kColorGreen, route_id_, i + 1,
                 points.size(), kColorReset);

        if (enable_gimbal_action_)
        {
          if (!notifyGimbalAndWait(p, i, points.size()))
          {
            return false;
          }
        }
      }
      else
      {
        ROS_ERROR("[GOAL FAILED] Route %d, Goal %zu/%zu, state: %s", route_id_, i + 1,
                  points.size(), state.toString().c_str());
        if (!continue_on_failure_)
        {
          return false;
        }
      }
    }

    ROS_INFO("%s[NAVIGATION FINISHED] Route %d completed%s", kColorGreen, route_id_,
             kColorReset);
    return true;
  }

} // namespace inspection_nav
