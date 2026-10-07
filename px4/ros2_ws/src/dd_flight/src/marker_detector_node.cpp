// Marker detector: finds the ArUco marker of the spot the drone was granted in the downward
// camera, turns its pose into the spot centre in PX4 local NED, and sends that to PX4 as the
// precision-landing target (MAVLink LANDING_TARGET).
//
// The target is static, so the node sends the mean of the last `window` estimates on every
// frame once it has `min_detections` of them, also on frames where the marker is out of view
// (close to the ground the marker, which sits beside the spot, leaves the image). This is the
// stationary-target assumption of PX4's own landing target estimator (spec A56).
//
// Seeded noise (run_flight.sh --seed): Gaussian noise of `corner_noise_px` on every detected
// corner before the pose is solved. Gazebo itself is not seeded.

#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <random>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "dd_flight/geometry.hpp"
#include "dd_flight/handshake.hpp"
#include "dd_flight/mavlink_target.hpp"
#include "dd_flight/px4_topics.hpp"
#include "dd_interfaces/msg/target_status.hpp"

using px4_msgs::msg::VehicleOdometry;
using dd_interfaces::msg::TargetStatus;

namespace dd_flight
{

class MarkerDetector : public rclcpp::Node
{
public:
  MarkerDetector()
  : Node("marker_detector")
  {
    side_m_ = declare_parameter<double>("marker_side_m");
    const auto offset = declare_parameter<std::vector<double>>("marker_offset_enu_m");
    const auto mount = declare_parameter<std::vector<double>>("camera_mount_flu_m");
    hfov_ = declare_parameter<double>("camera_hfov_rad");
    noise_px_ = declare_parameter<double>("corner_noise_px");
    window_ = static_cast<std::size_t>(declare_parameter<int>("window"));
    min_detections_ = static_cast<std::uint32_t>(declare_parameter<int>("min_detections"));
    rng_.seed(static_cast<std::uint64_t>(declare_parameter<int>("seed")));
    if (offset.size() != 2 || mount.size() != 3 || window_ == 0) {
      throw std::invalid_argument("marker_offset_enu_m needs 2 values, camera_mount_flu_m 3");
    }
    // Spot = marker - offset; the offset is a world (ENU) vector.
    offset_ned_ = enu_to_ned({offset[0], offset[1], 0.0});
    mount_frd_ = {mount[0], -mount[1], -mount[2]};
    mavlink_ = std::make_unique<MavlinkLandingTarget>(
      declare_parameter<std::string>("mavlink_host"),
      static_cast<std::uint16_t>(declare_parameter<int>("mavlink_port")));

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    params_ = cv::aruco::DetectorParameters::create();
    params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    odom_sub_ = create_subscription<VehicleOdometry>(
      px4_topic<VehicleOdometry>("/fmu/out/vehicle_odometry"), rclcpp::SensorDataQoS(),
      [this](const VehicleOdometry & m) {
        if (m.pose_frame == VehicleOdometry::POSE_FRAME_NED) {
          pos_ned_ = {m.position[0], m.position[1], m.position[2]};
          q_ = {m.q[0], m.q[1], m.q[2], m.q[3]};
          have_odom_ = std::isfinite(pos_ned_[0]) && std::isfinite(q_[0]);
        }
      });
    target_sub_ = create_subscription<std_msgs::msg::UInt16>(
      "/dd/detector/target_marker", HandshakeBus::qos(),
      [this](const std_msgs::msg::UInt16 & m) {
        marker_id_ = m.data;
        estimates_.clear();
        n_detections_ = 0;
        RCLCPP_INFO(get_logger(), "tracking marker %u", marker_id_);
      });
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/dd/down_cam", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image & m) {on_image(m);});
    status_pub_ = create_publisher<TargetStatus>("/dd/detector/status", 10);
  }

private:
  static constexpr std::uint16_t kNoMarker = 0xFFFF;

  void on_image(const sensor_msgs::msg::Image & img)
  {
    if (marker_id_ == kNoMarker || !have_odom_ || img.encoding != "rgb8") {
      return;
    }
    const cv::Mat rgb(static_cast<int>(img.height), static_cast<int>(img.width), CV_8UC3,
      const_cast<std::uint8_t *>(img.data.data()), img.step);
    cv::Mat gray;
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids, params_);

    TargetStatus status;
    status.t_us = sim_us(*this);
    status.marker_id = marker_id_;
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] != marker_id_) {
        continue;
      }
      std::normal_distribution<float> noise(0.0F, static_cast<float>(noise_px_));
      for (auto & c : corners[i]) {
        c.x += noise(rng_);
        c.y += noise(rng_);
      }
      status.detected = true;
      status.marker_px = static_cast<float>(cv::norm(corners[i][0] - corners[i][1]));
      add_estimate(corners[i], img.width, img.height);
    }
    status.n_detections = n_detections_;
    if (n_detections_ >= min_detections_) {
      const Vec3 t = mean();
      status.target_ned_m = {static_cast<float>(t[0]), static_cast<float>(t[1]),
        static_cast<float>(t[2])};
      if (!mavlink_->send(t, static_cast<std::uint8_t>(marker_id_))) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "LANDING_TARGET send failed");
      }
    }
    status_pub_->publish(status);
  }

  void add_estimate(const std::vector<cv::Point2f> & corners, std::uint32_t w, std::uint32_t h)
  {
    const double cx = static_cast<double>(w) / 2.0, cy = static_cast<double>(h) / 2.0;
    const double f = cx / std::tan(hfov_ / 2.0);
    const cv::Matx33d k(f, 0, cx, 0, f, cy, 0, 0, 1);
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(
      std::vector<std::vector<cv::Point2f>>{corners}, static_cast<float>(side_m_), k, cv::noArray(),
      rvecs, tvecs);
    const Vec3 marker = optical_to_ned({tvecs[0][0], tvecs[0][1], tvecs[0][2]}, mount_frd_, q_,
      pos_ned_);
    estimates_.push_back({marker[0] - offset_ned_[0], marker[1] - offset_ned_[1], marker[2]});
    if (estimates_.size() > window_) {
      estimates_.pop_front();
    }
    ++n_detections_;
  }

  [[nodiscard]] Vec3 mean() const
  {
    Vec3 m{0, 0, 0};
    for (const auto & e : estimates_) {
      for (int i = 0; i < 3; ++i) {
        m[i] += e[i] / static_cast<double>(estimates_.size());
      }
    }
    return m;
  }

  double side_m_{}, hfov_{}, noise_px_{};
  std::size_t window_{};
  std::uint32_t min_detections_{};
  Vec3 offset_ned_{}, mount_frd_{};
  Vec3 pos_ned_{};
  Quat q_{1, 0, 0, 0};
  bool have_odom_ = false;
  std::uint16_t marker_id_ = kNoMarker;
  std::uint32_t n_detections_ = 0;
  std::deque<Vec3> estimates_;
  std::mt19937_64 rng_;
  std::unique_ptr<MavlinkLandingTarget> mavlink_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<TargetStatus>::SharedPtr status_pub_;
};

}  // namespace dd_flight

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dd_flight::MarkerDetector>());
  rclcpp::shutdown();
  return 0;
}
