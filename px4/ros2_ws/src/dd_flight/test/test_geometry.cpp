// The camera chain is where a sign or axis mistake would put the landing target on the wrong
// side of the spot without any error; these pin it down against hand-worked cases.

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "dd_flight/geometry.hpp"

using dd_flight::Quat;
using dd_flight::Vec3;

namespace
{
constexpr double kTol = 1e-9;

Quat yaw(double rad)
{
  return {std::cos(rad / 2), 0, 0, std::sin(rad / 2)};
}

void expect_near(const Vec3 & a, const Vec3 & b)
{
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(a[i], b[i], kTol) << "component " << i;
  }
}
}  // namespace

TEST(Geometry, CameraOpticalAxesMapToBody)
{
  // Image right is body right, image down is body backward, the view is body down.
  expect_near(dd_flight::optical_to_frd({1, 0, 0}), {0, 1, 0});
  expect_near(dd_flight::optical_to_frd({0, 1, 0}), {-1, 0, 0});
  expect_near(dd_flight::optical_to_frd({0, 0, 1}), {0, 0, 1});
}

TEST(Geometry, MarkerStraightBelowLevelVehicle)
{
  // Level, facing north, 8 m up: a marker at the image centre is straight below the camera.
  const Vec3 p = dd_flight::optical_to_ned({0, 0, 8}, {0.15, 0, 0.03}, {1, 0, 0, 0}, {10, 20, -8});
  expect_near(p, {10.15, 20, 0.03});
}

TEST(Geometry, YawRotatesTheOffset)
{
  // Facing east (yaw +90 deg), something 1 m to the image's right is 1 m south.
  const Vec3 p = dd_flight::optical_to_ned({1, 0, 5}, {0, 0, 0}, yaw(std::numbers::pi / 2), {0, 0, -5});
  expect_near(p, {-1, 0, 0});
}

TEST(Geometry, RollTiltsTheView)
{
  // Rolled 90 deg right, the camera's view (body down) points west in NED.
  const Quat roll{std::cos(std::numbers::pi / 4), std::sin(std::numbers::pi / 4), 0, 0};
  expect_near(dd_flight::rotate(roll, {0, 0, 1}), {0, -1, 0});
}

TEST(Geometry, EnuToNed)
{
  expect_near(dd_flight::enu_to_ned({1, 2, 3}), {2, 1, -3});
}

TEST(Geometry, InducedPowerFollowsMomentumTheory)
{
  // T = 10 N on a 0.1 m rotor at sea level: P = T^1.5 / sqrt(2 rho A).
  const double k = 10.0 / (500.0 * 500.0);
  const double expected = std::pow(10.0, 1.5) / std::sqrt(2 * 1.225 * std::numbers::pi * 0.01);
  EXPECT_NEAR(dd_flight::induced_power_w(500.0, k, 1.225, 0.1), expected, 1e-9);
}
