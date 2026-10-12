// Ground-truth poses from Gazebo (bridged to /dd/truth/<model>), for the parts of the system
// that are hardware in reality: the pads' spot and pod presence sensors (spec A12) and the
// latch's capture of a pod. Flight control and the landing target never read them.
#pragma once

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dd_flight/geometry.hpp"

namespace dd_flight
{

class TruthPoses
{
public:
  TruthPoses(rclcpp::Node & node, const std::vector<std::string> & models)
  {
    for (const auto & m : models) {
      subs_.push_back(node.create_subscription<geometry_msgs::msg::Pose>(
          "/dd/truth/" + m, 10, [this, m](const geometry_msgs::msg::Pose & p) {
            enu_[m] = {p.position.x, p.position.y, p.position.z};
          }));
    }
  }

  [[nodiscard]] bool has(const std::string & model) const {return enu_.contains(model);}
  // World ENU position of the model origin; zeros until the first pose arrives.
  [[nodiscard]] Vec3 enu(const std::string & model) const
  {
    const auto it = enu_.find(model);
    return it == enu_.end() ? Vec3{0, 0, 0} : it->second;
  }
  [[nodiscard]] double horizontal_distance(const std::string & model, double x, double y) const
  {
    const Vec3 p = enu(model);
    return std::hypot(p[0] - x, p[1] - y);
  }

private:
  std::map<std::string, Vec3> enu_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr> subs_;
};

}  // namespace dd_flight
