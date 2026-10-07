// Sends MAVLink LANDING_TARGET to PX4 over UDP. PX4's MAVLink receiver turns a message with
// position_valid and frame LOCAL_NED into landing_target_pose, which navigator's precision
// land (AUTO_PRECLAND) steers to. This is PX4 v1.17's landing-target input that needs no change
// to the PX4 tree: its uXRCE-DDS bridge does not subscribe to landing_target_pose.
#pragma once

#include <cstdint>
#include <string>

#include "dd_flight/geometry.hpp"

namespace dd_flight
{

class MavlinkLandingTarget
{
public:
  // PX4 SITL's onboard MAVLink instance listens on UDP 14580.
  MavlinkLandingTarget(const std::string & host, std::uint16_t port);
  ~MavlinkLandingTarget();
  MavlinkLandingTarget(const MavlinkLandingTarget &) = delete;
  MavlinkLandingTarget & operator=(const MavlinkLandingTarget &) = delete;

  // Target position in PX4 local NED. time_usec 0: PX4 stamps it on arrival (its MAVLink
  // timesync is not converged in SITL), the same rule as /fmu/in messages.
  // Returns false if the datagram could not be sent.
  bool send(const Vec3 & target_ned_m, std::uint8_t target_num);

private:
  int fd_ = -1;
  std::string host_;
  std::uint16_t port_;
};

}  // namespace dd_flight
