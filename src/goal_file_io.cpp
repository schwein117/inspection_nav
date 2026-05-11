#include "inspection_nav/inspection_nav_common.hpp"

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <string>
#include <vector>

namespace {

int parseGimbalInspect(const YAML::Node& node) {
  if (!node || !node["gimbal_inspect"]) {
    return 0;
  }

  const YAML::Node flag = node["gimbal_inspect"];
  try {
    return flag.as<int>() != 0 ? 1 : 0;
  } catch (const std::exception&) {
  }

  try {
    return flag.as<bool>() ? 1 : 0;
  } catch (const std::exception&) {
  }

  return 0;
}

bool parseNavPoint(const YAML::Node& node, inspection_nav::NavPoint* p) {
  if (p == nullptr || !node || !node["position"] || !node["orientation"]) {
    return false;
  }

  const YAML::Node pos = node["position"];
  const YAML::Node ori = node["orientation"];
  if (!pos["x"] || !pos["y"] || !pos["z"] || !ori["x"] || !ori["y"] || !ori["z"] ||
      !ori["w"]) {
    return false;
  }

  try {
    p->x = pos["x"].as<double>();
    p->y = pos["y"].as<double>();
    p->z = pos["z"].as<double>();
    p->qx = ori["x"].as<double>();
    p->qy = ori["y"].as<double>();
    p->qz = ori["z"].as<double>();
    p->qw = ori["w"].as<double>();
  } catch (const std::exception&) {
    return false;
  }

  p->gimbal_inspect = parseGimbalInspect(node);
  return true;
}

YAML::Node toYamlNode(const inspection_nav::NavPoint& p, std::size_t index) {
  YAML::Node node;
  node["index"] = static_cast<int>(index + 1);
  node["name"] = "patrol_point_" + std::to_string(index + 1);
  node["gimbal_inspect"] = p.gimbal_inspect;
  node["position"]["x"] = p.x;
  node["position"]["y"] = p.y;
  node["position"]["z"] = p.z;
  node["orientation"]["x"] = p.qx;
  node["orientation"]["y"] = p.qy;
  node["orientation"]["z"] = p.qz;
  node["orientation"]["w"] = p.qw;
  return node;
}

bool savePointsYaml(const std::string& file_path,
                    const std::vector<inspection_nav::NavPoint>& points) {
  YAML::Node root;
  root["goals"] = YAML::Node(YAML::NodeType::Sequence);
  for (std::size_t i = 0; i < points.size(); ++i) {
    root["goals"].push_back(toYamlNode(points[i], i));
  }

  YAML::Emitter out;
  out.SetFloatPrecision(6);
  out << root;
  if (!out.good()) {
    ROS_ERROR("YAML serialization failed: %s", out.GetLastError().c_str());
    return false;
  }

  std::ofstream ofs(file_path.c_str(), std::ios::out | std::ios::trunc);
  if (!ofs.is_open()) {
    ROS_ERROR("Failed to open goal config for writing: %s", file_path.c_str());
    return false;
  }
  ofs << out.c_str() << "\n";
  return true;
}

}  // namespace

namespace inspection_nav {

const char* kDefaultConfigFile = "/home/schwein/inspection_ws/src/inspection_nav/config/point1.yaml";
const char* kDefaultGoalFrame = "map";

std::string buildRouteConfigPath(int route_id) {
  if (route_id <= 0) {
    route_id = 1;
  }
  return std::string("/home/schwein/inspection_ws/src/inspection_nav/config/point") +
         std::to_string(route_id) + ".yaml";
}

bool GoalFileIO::appendPoint(const std::string& file_path, const NavPoint& p) {
  std::vector<NavPoint> points;

  std::ifstream in(file_path.c_str());
  if (in.good() && in.peek() != std::ifstream::traits_type::eof()) {
    if (!GoalFileIO::loadPoints(file_path, &points)) {
      return false;
    }
  }

  points.push_back(p);
  if (!savePointsYaml(file_path, points)) {
    ROS_ERROR("Failed to write goal config file: %s", file_path.c_str());
    return false;
  }
  return true;
}

bool GoalFileIO::loadPoints(const std::string& file_path, std::vector<NavPoint>* points) {
  if (points == nullptr) {
    return false;
  }
  points->clear();

  YAML::Node root;
  try {
    root = YAML::LoadFile(file_path);
  } catch (const std::exception& e) {
    ROS_ERROR("Failed to read YAML config file: %s, error: %s", file_path.c_str(), e.what());
    return false;
  }

  if (!root || !root["goals"]) {
    return true;
  }

  const YAML::Node goals = root["goals"];
  if (!goals.IsSequence()) {
    ROS_ERROR("YAML field 'goals' is not a sequence: %s", file_path.c_str());
    return false;
  }

  for (std::size_t i = 0; i < goals.size(); ++i) {
    NavPoint p;
    if (!parseNavPoint(goals[i], &p)) {
      ROS_WARN("Skipping malformed goal entry: goals[%zu]", i);
      continue;
    }
    points->push_back(p);
  }

  return true;
}

}  // namespace inspection_nav
