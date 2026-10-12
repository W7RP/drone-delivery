#include "dd_flight/flight_records.hpp"

#include <bit>

namespace dd_flight
{

namespace
{
template<typename T, std::size_t N>
void put(std::array<std::uint8_t, N> & buf, std::size_t at, const T & v)
{
  static_assert(std::endian::native == std::endian::little, "records are little-endian");
  std::memcpy(buf.data() + at, &v, sizeof(T));
}
}  // namespace

std::array<std::uint8_t, kHeaderBytes> header(
  const char (&magic)[9], std::uint16_t minor, std::uint32_t count_a)
{
  std::array<std::uint8_t, kHeaderBytes> h{};
  std::memcpy(h.data(), magic, 8);
  put(h, 8, kVersionMajor);
  put(h, 10, minor);
  put(h, 12, static_cast<std::uint32_t>(kHeaderBytes));
  put(h, 16, count_a);
  put(h, 20, std::uint32_t{0});
  put(h, 24, std::uint64_t{0});
  return h;
}

std::array<std::uint8_t, kLaneRecordBytes> pack(const LaneRecord & r)
{
  std::array<std::uint8_t, kLaneRecordBytes> b{};
  put(b, 0, r.t_start_ms);
  put(b, 8, r.t_end_ms);
  put(b, 16, r.lane_id);
  put(b, 20, r.order_id);
  put(b, 24, r.payload_kg);
  put(b, 28, r.energy_j);
  put(b, 32, r.soc_start);
  put(b, 36, r.soc_end);
  put(b, 40, r.drone_id);
  put(b, 42, r.from_pad);
  put(b, 44, r.to_pad);
  put(b, 46, kSourcePx4Sitl);
  return b;
}

std::array<std::uint8_t, kSampleRecordBytes> pack(const SampleRecord & r)
{
  std::array<std::uint8_t, kSampleRecordBytes> b{};
  put(b, 0, r.t_us);
  for (std::size_t i = 0; i < 3; ++i) {
    put(b, 8 + 4 * i, r.pos_ned_m[i]);
    put(b, 20 + 4 * i, r.vel_ned_mps[i]);
    put(b, 76 + 4 * i, r.truth_enu_m[i]);
  }
  for (std::size_t i = 0; i < 4; ++i) {
    put(b, 32 + 4 * i, r.q_ned_frd[i]);
    put(b, 48 + 4 * i, r.motor_radps[i]);
  }
  put(b, 64, r.battery_v);
  put(b, 68, r.battery_a);
  put(b, 72, r.battery_remaining);
  put(b, 88, r.drone_mode);
  put(b, 89, r.nav_state);
  put(b, 90, r.arming_state);
  put(b, 91, r.pods_latched);
  return b;
}

}  // namespace dd_flight
