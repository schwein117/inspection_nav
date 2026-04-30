#include <ros/ros.h>

#include <string>
#include <vector>

#include "inspection_nav/goal_recorder.hpp"
#include "inspection_nav/inspection_nav_common.hpp"

namespace {

int parseRouteId(const std::vector<std::string>& args) {
  if (args.size() < 2) {
    return 1;
  }

  try {
    const int route_id = std::stoi(args[1]);
    if (route_id > 0) {
      return route_id;
    }
  } catch (const std::exception&) {
  }

  return 1;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "inspection_goal_recorder");
  ros::NodeHandle pnh("~");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);
  const int route_id = parseRouteId(args);

  std::string default_config = inspection_nav::buildRouteConfigPath(route_id);
  std::string config_file;
  pnh.param<std::string>("config_file", config_file, default_config);

  ROS_INFO("Recorder route selected: %d", route_id);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  inspection_nav::GoalRecorder recorder(pnh, config_file, route_id);
  recorder.run();
  return 0;
}
