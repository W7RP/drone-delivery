#include "dd_flight/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace dd_flight
{

Vec3 rotate(const Quat & q, const Vec3 & v)
{
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  // v' = q v q*, expanded.
  const double tx = 2.0 * (y * v[2] - z * v[1]);
  const double ty = 2.0 * (z * v[0] - x * v[2]);
  const double tz = 2.0 * (x * v[1] - y * v[0]);
  return {
    v[0] + w * tx + (y * tz - z * ty),
    v[1] + w * ty + (z * tx - x * tz),
    v[2] + w * tz + (x * ty - y * tx)};
}

Vec3 optical_to_frd(const Vec3 & p)
{
  return {-p[1], p[0], p[2]};
}

Vec3 optical_to_ned(
  const Vec3 & p_optical, const Vec3 & mount_frd, const Quat & q_ned_frd, const Vec3 & pos_ned)
{
  const Vec3 c = optical_to_frd(p_optical);
  const Vec3 body{mount_frd[0] + c[0], mount_frd[1] + c[1], mount_frd[2] + c[2]};
  const Vec3 r = rotate(q_ned_frd, body);
  return {pos_ned[0] + r[0], pos_ned[1] + r[1], pos_ned[2] + r[2]};
}

Vec3 px4_project(double lat_deg, double lon_deg, double ref_lat_deg, double ref_lon_deg)
{
  constexpr double kEarthRadiusM = 6371000.0;
  const double lat = lat_deg * std::numbers::pi / 180.0, lon = lon_deg * std::numbers::pi / 180.0;
  const double rlat = ref_lat_deg * std::numbers::pi / 180.0;
  const double rlon = ref_lon_deg * std::numbers::pi / 180.0;
  const double cos_d_lon = std::cos(lon - rlon);
  const double arg = std::clamp(std::sin(rlat) * std::sin(lat) +
      std::cos(rlat) * std::cos(lat) * cos_d_lon, -1.0, 1.0);
  const double c = std::acos(arg);
  const double k = std::abs(c) > 0 ? c / std::sin(c) : 1.0;
  return {k * (std::cos(rlat) * std::sin(lat) - std::sin(rlat) * std::cos(lat) * cos_d_lon) * kEarthRadiusM,
    k * std::cos(lat) * std::sin(lon - rlon) * kEarthRadiusM, 0.0};
}

Vec3 enu_to_ned(const Vec3 & enu)
{
  return {enu[1], enu[0], -enu[2]};
}

double induced_power_w(
  double omega_radps, double motor_constant, double air_density_kgpm3, double rotor_radius_m)
{
  const double thrust_n = motor_constant * omega_radps * omega_radps;
  const double disk_m2 = std::numbers::pi * rotor_radius_m * rotor_radius_m;
  return std::pow(thrust_n, 1.5) / std::sqrt(2.0 * air_density_kgpm3 * disk_m2);
}

double propeller_power_w(
  double omega_radps, double motor_constant, double air_density_kgpm3, double rotor_radius_m,
  double axial_speed_mps)
{
  const double thrust_n = motor_constant * omega_radps * omega_radps;
  const double disk_m2 = std::numbers::pi * rotor_radius_m * rotor_radius_m;
  const double v = std::max(axial_speed_mps, 0.0);
  const double vi = -0.5 * v + std::sqrt(0.25 * v * v + thrust_n / (2.0 * air_density_kgpm3 * disk_m2));
  return thrust_n * (v + vi);
}

}  // namespace dd_flight
