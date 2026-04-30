#pragma once

#include <string>
#include <vector>

namespace inspection_nav {

struct NavPoint {
  double x;
  double y;
  double z;
  double qx;
  double qy;
  double qz;
  double qw;
};

constexpr const char* kColorGreen = "\033[1;32m";
constexpr const char* kColorReset = "\033[0m";

extern const char* kDefaultConfigFile;
extern const char* kDefaultGoalFrame;

std::string buildRouteConfigPath(int route_id);

class GoalFileIO {
 public:
  static bool appendPoint(const std::string& file_path, const NavPoint& p);
  static bool loadPoints(const std::string& file_path, std::vector<NavPoint>* points);
};

}  // namespace inspection_nav
