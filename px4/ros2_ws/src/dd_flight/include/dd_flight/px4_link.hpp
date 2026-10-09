// The vehicle as the flight nodes see it: PX4 state in, offboard setpoints and commands out,
// all over the uXRCE-DDS /fmu topics (the interface a real flight controller offers a
// companion computer). Patterned on quad-autonomy-sim's offboard node: setpoints are streamed
// continuously while offboard is wanted, commands go out with timestamp 0 so PX4 stamps them on
// arrival, and the caller stops commanding the moment PX4 leaves the mode it asked for.
#pragma once

#include <cmath>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "dd_flight/geometry.hpp"
#include "dd_flight/px4_topics.hpp"

namespace dd_flight
{

class Px4Link
{
public:
  explicit Px4Link(rclcpp::Node & node)
  : node_(node)
  {
    using namespace px4_msgs::msg;
    const auto qos = rclcpp::SensorDataQoS();
    status_sub_ = node.create_subscription<VehicleStatus>(
      px4_topic<VehicleStatus>("/fmu/out/vehicle_status"), qos,
      [this](const VehicleStatus & m) {
        nav_state_ = m.nav_state;
        armed_ = m.arming_state == VehicleStatus::ARMING_STATE_ARMED;
        fixed_wing_ = m.vehicle_type == VehicleStatus::VEHICLE_TYPE_FIXED_WING;
        in_transition_ = m.in_transition_mode;
        have_status_ = true;
      });
    pos_sub_ = node.create_subscription<VehicleLocalPosition>(
      px4_topic<VehicleLocalPosition>("/fmu/out/vehicle_local_position"), qos,
      [this](const VehicleLocalPosition & m) {
        pos_ = {m.x, m.y, m.z};
        vel_ = {m.vx, m.vy, m.vz};
        heading_ = m.heading;
        pos_valid_ = m.xy_valid && m.z_valid && m.v_xy_valid && m.v_z_valid;
        if (m.xy_global) {
          ref_lat_ = m.ref_lat;
          ref_lon_ = m.ref_lon;
          have_ref_ = true;
        }
      });
    land_sub_ = node.create_subscription<VehicleLandDetected>(
      px4_topic<VehicleLandDetected>("/fmu/out/vehicle_land_detected"), qos,
      [this](const VehicleLandDetected & m) {landed_ = m.landed;});
    battery_sub_ = node.create_subscription<BatteryStatus>(
      px4_topic<BatteryStatus>("/fmu/out/battery_status"), qos,
      [this](const BatteryStatus & m) {soc_ = m.remaining;});
    mode_pub_ = node.create_publisher<OffboardControlMode>(
      px4_topic<OffboardControlMode>("/fmu/in/offboard_control_mode"), 10);
    sp_pub_ = node.create_publisher<TrajectorySetpoint>(
      px4_topic<TrajectorySetpoint>("/fmu/in/trajectory_setpoint"), 10);
    cmd_pub_ = node.create_publisher<VehicleCommand>(
      px4_topic<VehicleCommand>("/fmu/in/vehicle_command"), 10);
  }

  [[nodiscard]] bool ready() const {return have_status_ && pos_valid_ && have_ref_;}
  // A latitude/longitude in PX4's local NED frame (north, east; z left 0).
  [[nodiscard]] Vec3 local(double lat_deg, double lon_deg) const
  {
    return px4_project(lat_deg, lon_deg, ref_lat_, ref_lon_);
  }
  [[nodiscard]] bool armed() const {return armed_;}
  [[nodiscard]] bool landed() const {return landed_;}
  [[nodiscard]] bool offboard() const
  {
    return nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }
  [[nodiscard]] bool precland() const
  {
    return nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND;
  }
  [[nodiscard]] std::uint8_t nav_state() const {return nav_state_;}
  // VTOL: flying as a fixed wing, as a multicopter (also any non-VTOL), or between the two.
  [[nodiscard]] bool fixed_wing() const {return fixed_wing_ && !in_transition_;}
  [[nodiscard]] bool multicopter() const {return !fixed_wing_ && !in_transition_;}
  [[nodiscard]] bool in_transition() const {return in_transition_;}
  [[nodiscard]] const Vec3 & position() const {return pos_;}
  [[nodiscard]] const Vec3 & velocity() const {return vel_;}
  [[nodiscard]] double heading() const {return heading_;}
  [[nodiscard]] float soc() const {return soc_;}

  // One offboard cycle: position setpoint with a velocity feed-forward (NaN components are
  // ignored by PX4). Call at the control rate for as long as offboard is wanted.
  void setpoint(const Vec3 & pos_ned, const Vec3 & vel_ned, double yaw)
  {
    px4_msgs::msg::OffboardControlMode mode{};
    mode.position = true;
    mode.velocity = true;
    mode_pub_->publish(mode);
    px4_msgs::msg::TrajectorySetpoint sp{};
    for (int i = 0; i < 3; ++i) {
      sp.position[i] = static_cast<float>(pos_ned[i]);
      sp.velocity[i] = static_cast<float>(vel_ned[i]);
      sp.acceleration[i] = std::nanf("");
      sp.jerk[i] = std::nanf("");
    }
    sp.yaw = static_cast<float>(yaw);
    sp.yawspeed = std::nanf("");
    sp_pub_->publish(sp);
  }

  void request_offboard() {command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);}
  void arm() {command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);}
  void request_precland() {command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_PRECLAND);}
  // MAV_VTOL_STATE 4 (fixed wing) or 3 (multicopter).
  void transition(bool to_fixed_wing)
  {
    command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION,
      to_fixed_wing ? 4.0F : 3.0F);
  }

private:
  void command(std::uint16_t cmd, float p1 = 0.0F, float p2 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand c{};
    c.command = cmd;
    c.param1 = p1;
    c.param2 = p2;
    c.param5 = std::nan("");
    c.param6 = std::nan("");
    c.param7 = std::nanf("");
    c.target_system = 1;
    c.target_component = 1;
    c.source_system = 1;
    c.source_component = 1;
    c.from_external = true;
    cmd_pub_->publish(c);
  }

  rclcpp::Node & node_;
  std::uint8_t nav_state_ = 0;
  bool armed_ = false, landed_ = true, have_status_ = false, pos_valid_ = false, have_ref_ = false;
  bool fixed_wing_ = false, in_transition_ = false;
  double ref_lat_ = 0.0, ref_lon_ = 0.0;
  Vec3 pos_{}, vel_{};
  double heading_ = 0.0;
  float soc_ = std::nanf("");
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sp_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
};

}  // namespace dd_flight
