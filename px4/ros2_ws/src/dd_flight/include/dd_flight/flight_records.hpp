// Binary records of the SITL flight logs (docs/spec.md 9.3 flight_log.bin and 9.5
// flight_samples.bin). Little-endian, 32-byte spec 9.0 header, fixed-size records.
#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <string>

namespace dd_flight
{

constexpr std::size_t kHeaderBytes = 32;
constexpr std::uint16_t kVersionMajor = 1;
constexpr std::size_t kLaneRecordBytes = 48;
constexpr std::size_t kSampleRecordBytes = 96;
constexpr std::uint8_t kSourcePx4Sitl = 1;  // FlightSource PX4_SITL

// Spec 9.0 header; source_hash is 0 for PX4 calibration logs.
[[nodiscard]] std::array<std::uint8_t, kHeaderBytes> header(
  const char (&magic)[9], std::uint16_t minor, std::uint32_t count_a);

// Spec 9.3, one lane traversal.
struct LaneRecord
{
  std::uint64_t t_start_ms, t_end_ms;
  std::uint32_t lane_id, order_id;
  float payload_kg, energy_j, soc_start, soc_end;
  std::uint16_t drone_id, from_pad, to_pad;
};
[[nodiscard]] std::array<std::uint8_t, kLaneRecordBytes> pack(const LaneRecord & r);

// Spec 9.5, one sample.
struct SampleRecord
{
  std::uint64_t t_us;
  std::array<float, 3> pos_ned_m, vel_ned_mps;
  std::array<float, 4> q_ned_frd;
  std::array<float, 4> motor_radps;
  float battery_v, battery_a, battery_remaining;
  std::array<float, 3> truth_enu_m;
  std::uint8_t drone_mode, nav_state, arming_state, pods_latched;
};
[[nodiscard]] std::array<std::uint8_t, kSampleRecordBytes> pack(const SampleRecord & r);

}  // namespace dd_flight
