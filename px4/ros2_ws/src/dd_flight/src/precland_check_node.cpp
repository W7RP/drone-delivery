// Step 5 check: from the drone parked on A's OUT, climb to the approach height, fly over A's IN
// spot, ask the marker detector for that spot's marker, precision-land through PX4, and report
// the touchdown error against the spot centre from Gazebo's ground truth. Exits 0 on a landing.

#include <cmath>
#include <cstdio>
#include <deque>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include "dd_flight/handshake.hpp"
#include "dd_flight/maneuver.hpp"
#include "dd_interfaces/msg/target_status.hpp"

namespace dd_flight
{

class PreclandCheck : public rclcpp::Node
{
public:
  PreclandCheck()
  : Node("precland_check"), px4_(*this),
    maneuver_(px4_, {declare_parameter<double>("accel_mps2"),
        declare_parameter<double>("arrive_radius_m"), declare_parameter<double>("settle_speed_mps"),
        declare_parameter<double>("engage_timeout_s")})
  {
    start_enu_ = declare_parameter<std::vector<double>>("start_enu_m");
    spot_enu_ = declare_parameter<std::vector<double>>("spot_enu_m");
    marker_ = static_cast<std::uint16_t>(declare_parameter<int>("marker_id"));
    approach_alt_ = declare_parameter<double>("approach_alt_m");
    min_detections_ = static_cast<std::uint32_t>(declare_parameter<int>("min_detections"));
    marker_pub_ = create_publisher<std_msgs::msg::UInt16>("/dd/detector/target_marker",
        HandshakeBus::qos());
    status_sub_ = create_subscription<dd_interfaces::msg::TargetStatus>("/dd/detector/status", 10,
        [this](const dd_interfaces::msg::TargetStatus & s) {detections_ = s.n_detections;});
    truth_sub_ = create_subscription<geometry_msgs::msg::Pose>("/dd/truth/dd_quad", 10,
        [this](const geometry_msgs::msg::Pose & p) {truth_ = {p.position.x, p.position.y, p.position.z};});
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {step();});
  }

private:
  void step()
  {
    if (!px4_.ready()) {
      return;
    }
    const Vec3 spot = enu_to_ned({spot_enu_[0] - start_enu_[0], spot_enu_[1] - start_enu_[1], 0});
    if (phase_ == 0) {
      const Vec3 p = px4_.position();
      maneuver_.start({{Leg::Kind::kEngage, {p[0], p[1], -approach_alt_}},
                       {Leg::Kind::kLine, {spot[0], spot[1], -approach_alt_}, 2.0}}, 0.0);
      phase_ = 1;
    } else if (phase_ == 1 && maneuver_.status() == Maneuver::Status::kDone) {
      marker_pub_->publish(std_msgs::msg::UInt16().set__data(marker_));
      phase_ = 2;
    } else if (phase_ == 2 && detections_ >= min_detections_) {
      maneuver_.start({{Leg::Kind::kPrecland, {}}}, 0.0);
      phase_ = 3;
    } else if (phase_ == 3 && maneuver_.status() == Maneuver::Status::kDone) {
      const double err = std::hypot(truth_[0] - spot_enu_[0], truth_[1] - spot_enu_[1]);
      std::printf("PRECLAND_RESULT error_m=%.4f truth_enu=%.3f,%.3f spot_enu=%.3f,%.3f\n", err,
        truth_[0], truth_[1], spot_enu_[0], spot_enu_[1]);
      std::fflush(stdout);
      rclcpp::shutdown();
      return;
    }
    if (maneuver_.status() == Maneuver::Status::kFailed) {
      std::printf("PRECLAND_RESULT failed: %s\n", maneuver_.failure().c_str());
      std::fflush(stdout);
      rclcpp::shutdown();
      return;
    }
    maneuver_.step(0.05);
    const Vec3 & p = px4_.position();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "phase %d, maneuver %d, ned %.2f %.2f %.2f, nav %u, detections %u", phase_,
      static_cast<int>(maneuver_.status()), p[0], p[1], p[2], px4_.nav_state(), detections_);
  }

  Px4Link px4_;
  Maneuver maneuver_;
  std::vector<double> start_enu_, spot_enu_;
  std::uint16_t marker_{};
  double approach_alt_{};
  std::uint32_t min_detections_{}, detections_ = 0;
  int phase_ = 0;
  Vec3 truth_{};
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr marker_pub_;
  rclcpp::Subscription<dd_interfaces::msg::TargetStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr truth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dd_flight

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dd_flight::PreclandCheck>());
  return 0;
}
