// Frames and the marker geometry of the SITL flights. ROS-free so it can be unit tested.
//
// Frames: Gazebo world ENU (x east, y north, z up); PX4 local NED (x north, y east, z down)
// with its origin wherever EKF2 started; body FRD (x forward, y right, z down); the camera's
// OpenCV optical frame (x right, y down, z along the view). Quaternions are Hamilton, w-x-y-z,
// rotating body FRD vectors into NED (PX4 vehicle_odometry convention).
#pragma once

#include <array>

namespace dd_flight
{

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // w, x, y, z

[[nodiscard]] Vec3 rotate(const Quat & q, const Vec3 & v);

// The downward camera of tools/make_vehicle.py: a Gazebo camera link pitched +90 deg about
// body y, so the view points down and image up is body forward. Optical x (image right) is
// body right, optical y (image down) is body backward, optical z (view) is body down.
[[nodiscard]] Vec3 optical_to_frd(const Vec3 & p_optical);

// A point seen by the camera, in PX4 local NED: camera mount (FRD, from the body origin) plus
// the optical-frame point, rotated by the attitude and offset by the vehicle position.
[[nodiscard]] Vec3 optical_to_ned(
  const Vec3 & p_optical, const Vec3 & mount_frd, const Quat & q_ned_frd, const Vec3 & pos_ned);

// PX4's local frame from GPS: the azimuthal equidistant projection of PX4's MapProjection
// (src/lib/geo/geo.cpp, sphere of radius 6 371 000 m) about PX4's reference point. Pads are
// handed to the drone as latitude/longitude and placed in PX4's frame with this, as on a real
// vehicle; Gazebo's world ENU differs from it by about 0.26% in scale here (ellipsoid against
// sphere), 5 m over the 2 km lane (spec A61). Returns north, east, 0.
[[nodiscard]] Vec3 px4_project(double lat_deg, double lon_deg, double ref_lat_deg, double ref_lon_deg);

// ENU offsets (east, north, up) to NED offsets (north, east, down).
[[nodiscard]] Vec3 enu_to_ned(const Vec3 & enu);

// Ideal induced power of one rotor from momentum theory, in watts: thrust T = k_f * omega^2,
// P = T^1.5 / sqrt(2 * rho * A). It ignores profile drag and motor and ESC losses, so it is an
// energy proxy, not a power measurement (spec A57).
[[nodiscard]] double induced_power_w(
  double omega_radps, double motor_constant, double air_density_kgpm3, double rotor_radius_m);

}  // namespace dd_flight
