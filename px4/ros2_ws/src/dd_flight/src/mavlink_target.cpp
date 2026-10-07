#include "dd_flight/mavlink_target.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdexcept>

// PX4's generated MAVLink 2 headers (build/px4_sitl_default/mavlink), see setup_ws.sh.
#include <common/mavlink.h>

namespace dd_flight
{

namespace
{
// This node speaks as a companion computer on the vehicle (system 1).
constexpr std::uint8_t kSystemId = 1;
constexpr std::uint8_t kComponentId = MAV_COMP_ID_ONBOARD_COMPUTER;
}  // namespace

MavlinkLandingTarget::MavlinkLandingTarget(const std::string & host, std::uint16_t port)
: host_(host), port_(port)
{
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    throw std::runtime_error("cannot open a UDP socket for MAVLink");
  }
}

MavlinkLandingTarget::~MavlinkLandingTarget()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

bool MavlinkLandingTarget::send(const Vec3 & t, std::uint8_t target_num)
{
  mavlink_message_t msg;
  const float q[4] = {1.0F, 0.0F, 0.0F, 0.0F};
  mavlink_msg_landing_target_pack(
    kSystemId, kComponentId, &msg, 0, target_num, MAV_FRAME_LOCAL_NED,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    static_cast<float>(t[0]), static_cast<float>(t[1]), static_cast<float>(t[2]), q,
    LANDING_TARGET_TYPE_VISION_FIDUCIAL, 1);
  std::uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const auto len = mavlink_msg_to_send_buffer(buf, &msg);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port_);
  if (::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1) {
    return false;
  }
  const auto sent = ::sendto(fd_, buf, len, 0, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
  return sent == static_cast<ssize_t>(len);
}

}  // namespace dd_flight
