#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <string>
#include <vector>

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

class RoutePathPublisher {
 public:
  RoutePathPublisher(ros::NodeHandle& nh, ros::NodeHandle& pnh, int initial_route_id)
      : nh_(nh),
        pnh_(pnh),
        enabled_(false),
        route_id_(initial_route_id > 0 ? initial_route_id : 1),
        startup_route_id_(initial_route_id > 0 ? initial_route_id : 1) {
    pnh_.param<std::string>("path_topic", path_topic_, "/inspection_nav/patrol_path");
    pnh_.param<std::string>("path_frame", path_frame_, "map");
    pnh_.param<int>("startup_route_id", startup_route_id_, startup_route_id_);
    pnh_.param<bool>("publish_empty_on_startup", publish_empty_on_startup_, false);
    pnh_.param<bool>("publish_route_on_startup", publish_route_on_startup_, true);

    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
    enable_sub_ = nh_.subscribe("/inspection_nav/path_visualization/enable", 1,
                                &RoutePathPublisher::enableCb, this);
    route_sub_ = nh_.subscribe("/inspection_nav/path_visualization/route_id", 1,
                               &RoutePathPublisher::routeCb, this);

    ROS_INFO("Route path publisher started. Route: %d, topic: %s, frame: %s", route_id_,
             path_topic_.c_str(), path_frame_.c_str());

    if (publish_empty_on_startup_) {
      publishEmptyPath();
      ROS_INFO("Published empty startup path on %s", path_topic_.c_str());
    }

    if (publish_route_on_startup_) {
      route_id_ = startup_route_id_ > 0 ? startup_route_id_ : 1;
      enabled_ = true;
      publishRoutePath(route_id_);
    }
  }

 private:
  void enableCb(const std_msgs::Bool::ConstPtr& msg) {
    enabled_ = msg->data;
    if (enabled_) {
      publishRoutePath(route_id_);
    } else {
      publishEmptyPath();
      ROS_INFO("Path visualization disabled.");
    }
  }

  void routeCb(const std_msgs::Int32::ConstPtr& msg) {
    route_id_ = msg->data > 0 ? msg->data : 1;
    if (enabled_) {
      publishRoutePath(route_id_);
    }
  }

  void publishRoutePath(int route_id) {
    const std::string config_file = inspection_nav::buildRouteConfigPath(route_id);

    std::vector<inspection_nav::NavPoint> points;
    if (!inspection_nav::GoalFileIO::loadPoints(config_file, &points)) {
      ROS_WARN("Failed to load route %d config: %s", route_id, config_file.c_str());
      publishEmptyPath();
      return;
    }

    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = path_frame_;

    for (std::size_t i = 0; i < points.size(); ++i) {
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = points[i].x;
      pose.pose.position.y = points[i].y;
      pose.pose.position.z = points[i].z;
      pose.pose.orientation.x = points[i].qx;
      pose.pose.orientation.y = points[i].qy;
      pose.pose.orientation.z = points[i].qz;
      pose.pose.orientation.w = points[i].qw;
      path.poses.push_back(pose);
    }

    path_pub_.publish(path);
    ROS_INFO("%s[PATH PUBLISHED] Route %d, %zu points -> %s%s", inspection_nav::kColorGreen,
             route_id, path.poses.size(), path_topic_.c_str(), inspection_nav::kColorReset);
  }

  void publishEmptyPath() {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = path_frame_;
    path_pub_.publish(path);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher path_pub_;
  ros::Subscriber enable_sub_;
  ros::Subscriber route_sub_;

  bool enabled_;
  int route_id_;
  int startup_route_id_;
  bool publish_empty_on_startup_;
  bool publish_route_on_startup_;
  std::string path_topic_;
  std::string path_frame_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "inspection_route_path_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);
  const int route_id = parseRouteId(args);

  RoutePathPublisher node(nh, pnh, route_id);
  ros::spin();
  return 0;
}
