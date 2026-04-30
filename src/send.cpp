#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <string>
#include <vector>

#include "inspection_nav/goal_sender.hpp"
#include "inspection_nav/inspection_nav_common.hpp"

namespace
{

  int parseRouteId(const std::vector<std::string> &args)
  {
    if (args.size() < 2)
    {
      return 1;
    }

    try
    {
      const int route_id = std::stoi(args[1]);
      if (route_id > 0)
      {
        return route_id;
      }
    }
    catch (const std::exception &)
    {
    }

    return 1;
  }

} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inspection_goal_sender");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);
  const int route_id = parseRouteId(args);

  std::string default_config = inspection_nav::buildRouteConfigPath(route_id);
  std::string config_file;
  std::string goal_frame;
  double goal_timeout_sec = 0.0;
  bool continue_on_failure = false;
  bool enable_path_visualization = false;
  bool enable_gimbal_action = false;

  pnh.param<std::string>("config_file", config_file, default_config);
  pnh.param<std::string>("goal_frame", goal_frame, inspection_nav::kDefaultGoalFrame);
  pnh.param<double>("goal_timeout_sec", goal_timeout_sec, 0.0);
  pnh.param<bool>("continue_on_failure", continue_on_failure, false);
  pnh.param<bool>("enable_path_visualization", enable_path_visualization, false);
  pnh.param<bool>("enable_gimbal_action", enable_gimbal_action, false);

  ROS_INFO("Sender route selected: %d", route_id);
  ROS_INFO("Path visualization: %s", enable_path_visualization ? "enabled" : "disabled");

  ros::Publisher enable_pub =
      nh.advertise<std_msgs::Bool>("/inspection_nav/path_visualization/enable", 1, true);
  ros::Publisher route_pub =
      nh.advertise<std_msgs::Int32>("/inspection_nav/path_visualization/route_id", 1, true);

  std_msgs::Int32 route_msg;
  route_msg.data = route_id;
  route_pub.publish(route_msg);

  std_msgs::Bool enable_msg;
  enable_msg.data = enable_path_visualization;
  enable_pub.publish(enable_msg);

  // Give subscribers a short moment to receive the latched command.
  ros::Duration(0.2).sleep();
  ros::spinOnce();

  inspection_nav::GoalSender sender(pnh, config_file, goal_frame, goal_timeout_sec,
                                    continue_on_failure, route_id, enable_gimbal_action);
  return sender.run() ? 0 : 2;
}
