// Flight logger. Writes into <run_dir>:
//   events.jsonl        every handshake message and state transition, sim-time stamped; LANDED
//                       messages also carry the drone's ground-truth position (for landing error)
//   flight_samples.bin  spec 9.5: one 96-byte record per sample_period_s of sim time: PX4
//                       position, velocity, attitude, battery, the motor speeds PX4 commands,
//                       ground truth, drone mode and PX4 state
//   flight_log.bin      spec 9.3: one 48-byte record per lane flown (source PX4_SITL), with the
//                       energy proxy (geometry.hpp induced_power_w) integrated over the lane
//   visits.json         per visit: time in each state (pad visits and the drone), landing error
//                       at each touchdown against the spot centre
//   run_status          the run's end, which run_flight.sh waits for
// The energy proxy is integrated from the motor-speed commands, each held until the next one.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <actuator_msgs/msg/actuators.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "dd_flight/flight_records.hpp"
#include "dd_flight/geometry.hpp"
#include "dd_flight/handshake.hpp"
#include "dd_flight/px4_topics.hpp"

namespace dd_flight
{

// Every message name of spec 4.2 that the SITL nodes exchange.
const std::vector<std::string> kMessages = {
  "LANDING_REQUEST", "LANDING_CANCEL", "APPROACH_ABORT", "LANDED", "RELEASE_REPORT",
  "LATCH_REPORT", "DEPART_REQUEST", "DEPARTED", "LANDING_GRANT", "LANDING_HOLD",
  "LANDING_REFUSED", "RELEASE_CMD", "HOP_CMD", "LATCH_CMD", "CHARGE_START", "SERVICE_COMPLETE",
  "DEPART_CLEAR", "DRONE_STATUS", "MISSION_REQUEST", "HOLD_REPORT", "MISSION_ASSIGN",
  "HOLD_CONTINUE", "POD_PLACED", "POD_LATCHED", "POD_DELIVERED", "POD_REMOVED", "PAD_STATUS",
  "SHIPMENT_PENDING", "PICKUP_ASSIGNED", "POD_RETURN_REQUEST"};

std::string json_number(double v)
{
  if (!std::isfinite(v)) {return "null";}
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return buf;
}

std::string json_id(std::uint64_t v, std::uint64_t none)
{
  return v == none ? "null" : std::to_string(v);
}

// DroneMode codes (schemas/common.schema.json).
std::uint8_t mode_code(const std::string & mode)
{
  static const std::vector<std::string> modes = {"PARKED", "ON_PAD", "TAKEOFF", "EN_ROUTE",
    "HOLDING", "APPROACH", "GO_AROUND", "HOPPING", "GROUNDED"};
  for (std::size_t i = 0; i < modes.size(); ++i) {
    if (modes[i] == mode) {return static_cast<std::uint8_t>(i);}
  }
  return kNoCode;
}

struct StateTime
{
  std::string key;     // "VISIT pad 4 drone 0" or "DRONE 0"
  std::string state;
  std::uint64_t enter_us;
};

class FlightLogger : public rclcpp::Node
{
public:
  FlightLogger()
  : Node("flight_logger"), bus_(*this, "LOGGER")
  {
    run_dir_ = declare_parameter<std::string>("run_dir");
    lane_.lane_id = static_cast<std::uint32_t>(declare_parameter<int>("lane_id"));
    lane_.order_id = static_cast<std::uint32_t>(declare_parameter<int>("order_id"));
    lane_.payload_kg = static_cast<float>(declare_parameter<double>("payload_kg"));
    lane_.drone_id = static_cast<std::uint16_t>(declare_parameter<int>("drone_id"));
    lane_.from_pad = static_cast<std::uint16_t>(declare_parameter<int>("from_pad"));
    lane_.to_pad = static_cast<std::uint16_t>(declare_parameter<int>("to_pad"));
    motor_constant_ = declare_parameter<double>("motor_constant");
    rotor_radius_m_ = declare_parameter<double>("rotor_radius_m");
    air_density_ = declare_parameter<double>("air_density_kgpm3");
    const auto spot_pads = declare_parameter<std::vector<int64_t>>("spot_pad_ids");
    const auto spot_codes = declare_parameter<std::vector<int64_t>>("spot_codes");
    const auto spot_enu = declare_parameter<std::vector<double>>("spot_enu_m");
    for (std::size_t i = 0; i < spot_pads.size(); ++i) {
      spots_.push_back({static_cast<std::uint16_t>(spot_pads[i]),
          static_cast<std::uint8_t>(spot_codes[i]), spot_enu[2 * i], spot_enu[2 * i + 1]});
    }
    const double period_s = declare_parameter<double>("sample_period_s");

    events_.open(run_dir_ + "/events.jsonl");
    samples_.open(run_dir_ + "/flight_samples.bin", std::ios::binary);
    if (!events_ || !samples_) {
      throw std::runtime_error("cannot write the logs in " + run_dir_);
    }
    const auto h = header("DDSAMPL\0", 0, static_cast<std::uint32_t>(kSampleRecordBytes));
    samples_.write(reinterpret_cast<const char *>(h.data()), h.size());

    using namespace px4_msgs::msg;
    const auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<VehicleOdometry>(px4_topic<VehicleOdometry>(
          "/fmu/out/vehicle_odometry"), qos, [this](const VehicleOdometry & m) {
          for (std::size_t i = 0; i < 3; ++i) {
            sample_.pos_ned_m[i] = m.position[i];
            sample_.vel_ned_mps[i] = m.velocity[i];
          }
          for (std::size_t i = 0; i < 4; ++i) {sample_.q_ned_frd[i] = m.q[i];}
        });
    battery_sub_ = create_subscription<BatteryStatus>(px4_topic<BatteryStatus>(
          "/fmu/out/battery_status"), qos, [this](const BatteryStatus & m) {
          sample_.battery_v = m.voltage_v;
          sample_.battery_a = m.current_a;
          sample_.battery_remaining = m.remaining;
        });
    status_sub_ = create_subscription<VehicleStatus>(px4_topic<VehicleStatus>(
          "/fmu/out/vehicle_status"), qos, [this](const VehicleStatus & m) {
          sample_.nav_state = m.nav_state;
          sample_.arming_state = m.arming_state;
        });
    motor_sub_ = create_subscription<actuator_msgs::msg::Actuators>("/dd/motor_speed", qos,
        [this](const actuator_msgs::msg::Actuators & m) {on_motors(m);});
    truth_sub_ = create_subscription<geometry_msgs::msg::Pose>("/dd/truth/dd_quad", 10,
        [this](const geometry_msgs::msg::Pose & p) {
          truth_ = {p.position.x, p.position.y, p.position.z};
        });
    sample_.drone_mode = mode_code("PARKED");
    sample_.pods_latched = 1;
    sample_timer_ = rclcpp::create_timer(this, get_clock(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s)),
        [this]() {write_sample();});
    for (const auto & name : kMessages) {
      bus_.on(name, [this](const Handshake & m) {log_message(m);});
    }
    transitions_ = create_subscription<Transition>("/dd/transitions", HandshakeBus::qos(),
        [this](const Transition & t) {log_transition(t);});
    status_ = create_subscription<std_msgs::msg::String>("/dd/run_status",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::String & s) {finish(s.data);});
  }

private:
  struct Spot
  {
    std::uint16_t pad;
    std::uint8_t code;
    double x, y;
  };

  // Motor speeds as commanded by PX4 (rad/s). Each command holds until the next; the energy
  // proxy integrates the induced power of the four rotors over that time.
  void on_motors(const actuator_msgs::msg::Actuators & m)
  {
    const std::uint64_t t = sim_us(*this);
    if (last_motor_us_ != 0 && t > last_motor_us_) {
      energy_j_ += power_w_ * static_cast<double>(t - last_motor_us_) * 1e-6;
    }
    last_motor_us_ = t;
    power_w_ = 0.0;
    for (std::size_t i = 0; i < 4 && i < m.velocity.size(); ++i) {
      sample_.motor_radps[i] = static_cast<float>(m.velocity[i]);
      power_w_ += induced_power_w(m.velocity[i], motor_constant_, air_density_, rotor_radius_m_);
    }
  }

  void write_sample()
  {
    const std::uint64_t t = sim_us(*this);
    // Strictly increasing times (spec 9.5): the final sample at the end can fall on the last
    // timer sample's time.
    if (finished_ || t <= sample_.t_us) {return;}
    sample_.t_us = t;
    for (std::size_t i = 0; i < 3; ++i) {sample_.truth_enu_m[i] = static_cast<float>(truth_[i]);}
    const auto b = pack(sample_);
    samples_.write(reinterpret_cast<const char *>(b.data()), b.size());
  }

  void log_message(const Handshake & m)
  {
    events_ << "{\"type\":\"msg\",\"t_us\":" << m.sent_t_us << ",\"name\":\"" << m.name
            << "\",\"sender\":\"" << m.sender << "\",\"seq\":" << m.msg_seq
            << ",\"drone_id\":" << json_id(m.drone_id, kNoId16)
            << ",\"pad_id\":" << json_id(m.pad_id, kNoId16)
            << ",\"pod_id\":" << json_id(m.pod_id, kNoId32)
            << ",\"order_id\":" << json_id(m.order_id, kNoId32)
            << ",\"dest_pad\":" << json_id(m.dest_pad, kNoId16)
            << ",\"spot\":" << json_id(m.spot, kNoCode)
            << ",\"soc\":" << json_number(m.soc) << ",\"attempt\":" << int{m.attempt}
            << ",\"flag\":" << (m.flag ? "true" : "false") << ",\"code\":\"" << m.code << "\"";
    if (m.name == "LANDED") {
      events_ << ",\"truth_enu_m\":[" << json_number(truth_[0]) << "," << json_number(truth_[1])
              << "," << json_number(truth_[2]) << "]";
      landings_.push_back({m.pad_id, m.spot, truth_[0], truth_[1], m.sent_t_us});
    }
    if (m.name == "RELEASE_REPORT" && m.flag) {sample_.pods_latched = 0;}
    if (m.name == "LATCH_REPORT" && m.flag) {sample_.pods_latched = 1;}
    events_ << "}\n";
    events_.flush();
  }

  void log_transition(const Transition & t)
  {
    events_ << "{\"type\":\"transition\",\"t_us\":" << t.t_us << ",\"machine\":\"" << t.machine
            << "\",\"row\":\"" << t.row << "\",\"pad_id\":" << json_id(t.pad_id, kNoId16)
            << ",\"drone_id\":" << json_id(t.drone_id, kNoId16) << ",\"from\":\"" << t.from_state
            << "\",\"to\":\"" << t.to_state << "\"}\n";
    events_.flush();
    if (t.machine == "DRONE") {
      sample_.drone_mode = mode_code(t.to_state);
      if (t.row == "D02") {  // top of climb: the lane starts
        lane_.t_start_ms = t.t_us / 1000;
        lane_.soc_start = sample_.battery_remaining;
        lane_energy_start_j_ = energy_j_;
      } else if (t.row == "D03") {  // holding fix of the destination: the lane ends
        lane_.t_end_ms = t.t_us / 1000;
        lane_.soc_end = sample_.battery_remaining;
        lane_.energy_j = static_cast<float>(energy_j_ - lane_energy_start_j_);
        lanes_.push_back(lane_);
      }
    }
    if (t.machine == "DRONE" || t.machine == "VISIT") {
      const std::string key = t.machine == "DRONE" ? "drone " + std::to_string(t.drone_id) :
        "pad " + std::to_string(t.pad_id) + " drone " + std::to_string(t.drone_id);
      states_.push_back({key, t.to_state, t.t_us});
    }
  }

  void write_flight_log()
  {
    std::ofstream f(run_dir_ + "/flight_log.bin", std::ios::binary);
    const auto h = header("DDFLOG\0\0", 0, static_cast<std::uint32_t>(kLaneRecordBytes));
    f.write(reinterpret_cast<const char *>(h.data()), h.size());
    for (const auto & l : lanes_) {
      const auto b = pack(l);
      f.write(reinterpret_cast<const char *>(b.data()), b.size());
    }
  }

  // Time in each state per machine instance (the last state runs until the end), and landing
  // error at each touchdown against the centre of the spot it reported.
  void write_visits(std::uint64_t end_us)
  {
    std::ofstream f(run_dir_ + "/visits.json");
    f << "{\n \"end_t_us\": " << end_us << ",\n \"state_durations\": [";
    bool first = true;
    for (std::size_t i = 0; i < states_.size(); ++i) {
      std::uint64_t until = end_us;
      for (std::size_t j = i + 1; j < states_.size(); ++j) {
        if (states_[j].key == states_[i].key) {until = states_[j].enter_us; break;}
      }
      f << (first ? "\n  " : ",\n  ") << "{\"machine\": \"" << states_[i].key
        << "\", \"state\": \"" << states_[i].state << "\", \"enter_t_us\": "
        << states_[i].enter_us << ", \"duration_s\": "
        << json_number(static_cast<double>(until - states_[i].enter_us) * 1e-6) << "}";
      first = false;
    }
    f << "\n ],\n \"landings\": [";
    first = true;
    for (const auto & l : landings_) {
      double err = std::nan("");
      for (const auto & s : spots_) {
        if (s.pad == l.pad && s.code == l.spot) {err = std::hypot(l.x - s.x, l.y - s.y);}
      }
      f << (first ? "\n  " : ",\n  ") << "{\"pad_id\": " << l.pad << ", \"spot\": \""
        << (l.spot == kSpotIn ? "IN" : "OUT") << "\", \"t_us\": " << l.t_us
        << ", \"truth_enu_m\": [" << json_number(l.x) << ", " << json_number(l.y)
        << "], \"error_cm\": " << json_number(err * 100.0) << "}";
      first = false;
    }
    f << "\n ]\n}\n";
  }

  void finish(const std::string & status)
  {
    if (finished_) {return;}
    write_sample();
    finished_ = true;
    const std::uint64_t end_us = sim_us(*this);
    events_ << "{\"type\":\"run_status\",\"t_us\":" << end_us << ",\"status\":\"" << status
            << "\"}\n";
    events_.close();
    samples_.close();
    write_flight_log();
    write_visits(end_us);
    std::ofstream(run_dir_ + "/run_status") << status << "\n";
    RCLCPP_INFO(get_logger(), "run ended: %s", status.c_str());
  }

  struct Landing
  {
    std::uint16_t pad;
    std::uint8_t spot;
    double x, y;
    std::uint64_t t_us;
  };

  HandshakeBus bus_;
  std::string run_dir_;
  std::ofstream events_, samples_;
  bool finished_ = false;
  SampleRecord sample_{};
  LaneRecord lane_{};
  std::vector<LaneRecord> lanes_;
  std::vector<Spot> spots_;
  std::vector<Landing> landings_;
  std::vector<StateTime> states_;
  Vec3 truth_{};
  double motor_constant_{}, rotor_radius_m_{}, air_density_{};
  double energy_j_ = 0.0, power_w_ = 0.0, lane_energy_start_j_ = 0.0;
  std::uint64_t last_motor_us_ = 0;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr motor_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr truth_sub_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
  rclcpp::Subscription<Transition>::SharedPtr transitions_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_;
};

}  // namespace dd_flight

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dd_flight::FlightLogger>());
  rclcpp::shutdown();
  return 0;
}
