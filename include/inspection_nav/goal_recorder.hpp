#pragma once

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <atomic>
#include <mutex>
#include <string>

#include "inspection_nav/inspection_nav_common.hpp"

namespace inspection_nav {

class GoalRecorder {
 public:
  GoalRecorder(ros::NodeHandle& nh, const std::string& config_file, int route_id);
  void run();

 private:
  void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
  void clickedPointCb(const geometry_msgs::PointStamped::ConstPtr& msg);
  void keyboardLoop();

  bool getLatestOrientation(double* qx, double* qy, double* qz, double* qw);
  bool getLatestOdomPoint(NavPoint* p);
  bool writePoint(const NavPoint& p);

  ros::NodeHandle nh_;
  std::string config_file_;
  int route_id_;

  ros::Subscriber clicked_sub_;
  ros::Subscriber odom_sub_;

  nav_msgs::Odometry latest_odom_;
  std::mutex odom_mtx_;
  std::mutex file_mtx_;

  bool has_odom_;
  std::atomic<bool> running_;
};

}  // namespace inspection_nav
