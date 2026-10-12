// Drone agent: the drone side of the spec handshake (docs/spec.md 4.4) for the normal delivery
// visit, flying the vehicle through PX4 offboard and precision land.
//
// VTOL (airframe vtol, Phase 7): the lane is flown as a fixed wing between a transition after
// the climb and a back-transition back_transition_m before the destination's holding fix; the
// spec rows are the same.
//
// Rows implemented: D19 (mission -> DEPART_REQUEST), D01 (DEPART_CLEAR -> TAKEOFF), D02
// (top of climb -> EN_ROUTE, DEPARTED), D03 (holding fix reached -> HOLDING, LANDING_REQUEST),
// D04 (LANDING_GRANT -> APPROACH), D05 (touchdown with the marker detected -> ON_PAD, LANDED),
// D15 (RELEASE_CMD, HOP_CMD, LATCH_CMD and their completions), D17 (SERVICE_COMPLETE ->
// MISSION_REQUEST), D18 (PARK on OUT -> PARKED). Failure rows are out of scope (Phase 4 flies
// the normal path only): anything unexpected ends the run as FAILED and stops commanding.
//
// Timing: all durations are sim time (use_sim_time, Gazebo clock). Release and latch take the
// scenario's pad_ops.release_s and latch_s; flight legs take what the vehicle needs.

#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include "dd_flight/handshake.hpp"
#include "dd_flight/maneuver.hpp"
#include "dd_flight/sim_sensors.hpp"
#include "dd_interfaces/msg/target_status.hpp"

namespace dd_flight
{

enum class Mode {kParked, kOnPad, kTakeoff, kEnRoute, kHolding, kApproach, kHopping, kEnded};

const char * name(Mode m)
{
  switch (m) {
    case Mode::kParked: return "PARKED";
    case Mode::kOnPad: return "ON_PAD";
    case Mode::kTakeoff: return "TAKEOFF";
    case Mode::kEnRoute: return "EN_ROUTE";
    case Mode::kHolding: return "HOLDING";
    case Mode::kApproach: return "APPROACH";
    case Mode::kHopping: return "HOPPING";
    case Mode::kEnded: return "ENDED";
  }
  return "?";
}

// One pad of the corridor: latitude/longitude (degrees) of its centre and spots, and markers.
struct LatLon
{
  double lat, lon;
};

struct PadGeo
{
  std::uint16_t id;
  LatLon center, in, out;
  std::uint16_t in_marker, out_marker;
};

class DroneAgent : public rclcpp::Node
{
public:
  DroneAgent()
  : Node("drone_agent"), bus_(*this, "DRONE"), px4_(*this),
    maneuver_(px4_, {declare_parameter<double>("accel_mps2"),
        declare_parameter<double>("arrive_radius_m"), declare_parameter<double>("settle_speed_mps"),
        declare_parameter<double>("engage_timeout_s"),
        declare_parameter<double>("transition_timeout_s", 40.0)}),
    truth_(*this, {"drone", "dd_pod_1", "dd_pod_2"})
  {
    id_ = static_cast<std::uint16_t>(declare_parameter<int>("drone_id"));
    const auto ids = declare_parameter<std::vector<int64_t>>("pad_ids");
    const auto centers = declare_parameter<std::vector<double>>("pad_center_latlon_deg");
    const auto ins = declare_parameter<std::vector<double>>("pad_in_latlon_deg");
    const auto outs = declare_parameter<std::vector<double>>("pad_out_latlon_deg");
    const auto in_markers = declare_parameter<std::vector<int64_t>>("pad_in_marker");
    const auto out_markers = declare_parameter<std::vector<int64_t>>("pad_out_marker");
    for (std::size_t i = 0; i < ids.size(); ++i) {
      pads_.push_back({static_cast<std::uint16_t>(ids[i]), {centers[2 * i], centers[2 * i + 1]},
          {ins[2 * i], ins[2 * i + 1]}, {outs[2 * i], outs[2 * i + 1]},
          static_cast<std::uint16_t>(in_markers[i]), static_cast<std::uint16_t>(out_markers[i])});
    }
    pad_ = static_cast<std::uint16_t>(declare_parameter<int>("start_pad"));
    pod_ = static_cast<std::uint32_t>(declare_parameter<int>("start_pod"));
    order_ = static_cast<std::uint32_t>(declare_parameter<int>("start_order"));
    cruise_speed_ = declare_parameter<double>("cruise_speed_mps");
    cruise_alt_ = declare_parameter<double>("cruise_alt_m");
    approach_alt_ = declare_parameter<double>("approach_alt_m");
    hop_alt_ = declare_parameter<double>("hop_alt_m");
    hop_speed_ = declare_parameter<double>("hop_speed_mps");
    release_s_ = declare_parameter<double>("release_s");
    latch_s_ = declare_parameter<double>("latch_s");
    capture_m_ = declare_parameter<double>("latch_capture_radius_m");
    min_detections_ = static_cast<std::uint32_t>(declare_parameter<int>("min_detections"));
    target_timeout_s_ = declare_parameter<double>("target_timeout_s");
    vtol_ = declare_parameter<std::string>("airframe", "quad") == "vtol";
    back_transition_m_ = declare_parameter<double>("back_transition_m", 0.0);

    bus_.on("MISSION_ASSIGN", [this](const Handshake & m) {on_mission(m);});
    bus_.on("DEPART_CLEAR", [this](const Handshake & m) {on_depart_clear(m);});
    bus_.on("LANDING_GRANT", [this](const Handshake & m) {on_grant(m);});
    bus_.on("RELEASE_CMD", [this](const Handshake & m) {on_release_cmd(m);});
    bus_.on("HOP_CMD", [this](const Handshake & m) {on_hop_cmd(m);});
    bus_.on("LATCH_CMD", [this](const Handshake & m) {on_latch_cmd(m);});
    bus_.on("SERVICE_COMPLETE", [this](const Handshake & m) {on_service_complete(m);});

    marker_pub_ = create_publisher<std_msgs::msg::UInt16>("/dd/detector/target_marker",
        HandshakeBus::qos());
    status_pub_ = create_publisher<std_msgs::msg::String>("/dd/run_status",
        rclcpp::QoS(1).reliable().transient_local());
    for (const char * pod : {"dd_pod_1", "dd_pod_2"}) {
      latch_pubs_[std::string(pod) + "/attach"] =
        create_publisher<std_msgs::msg::Empty>(std::string("/dd/") + pod + "/attach", 10);
      latch_pubs_[std::string(pod) + "/detach"] =
        create_publisher<std_msgs::msg::Empty>(std::string("/dd/") + pod + "/detach", 10);
    }
    detector_sub_ = create_subscription<dd_interfaces::msg::TargetStatus>("/dd/detector/status",
        10, [this](const dd_interfaces::msg::TargetStatus & s) {
          if (s.marker_id == awaited_marker_) {detections_ = s.n_detections;}
        });
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(50),
        [this]() {step();});
  }

private:
  // Approach and hop: fly to the spot, wait for the detector to lock on its marker, land.
  enum class Landing {kNone, kFlyToSpot, kWaitTarget, kLand};

  [[nodiscard]] const PadGeo & pad(std::uint16_t id) const
  {
    for (const auto & p : pads_) {
      if (p.id == id) {return p;}
    }
    throw std::out_of_range("unknown pad id " + std::to_string(id));
  }

  // A pad location at a height above ground, in PX4 local NED. Heights are above the EKF
  // origin, which is on the ground at the start (the world is flat, A54).
  [[nodiscard]] Vec3 ned(const LatLon & p, double alt) const
  {
    const Vec3 d = px4_.local(p.lat, p.lon);
    return {d[0], d[1], -alt};
  }

  void go(Mode to, const std::string & row)
  {
    bus_.transition("DRONE", row, pad_, id_, name(mode_), name(to));
    mode_ = to;
  }

  [[nodiscard]] Handshake to_pad(const std::string & msg_name, std::uint16_t pad_id) const
  {
    Handshake m = blank(msg_name);
    m.drone_id = id_;
    m.pad_id = pad_id;
    return m;
  }

  void drone_status(const std::string & mode)
  {
    Handshake m = to_pad("DRONE_STATUS", pad_);
    m.code = mode;
    m.dest_pad = dest_;
    m.pod_id = pod_;
    m.soc = px4_.soc();
    bus_.send(m);
  }

  void end_run(const std::string & status)
  {
    if (mode_ == Mode::kEnded) {return;}
    RCLCPP_INFO(get_logger(), "run %s", status.c_str());
    status_pub_->publish(std_msgs::msg::String().set__data(status));
    ended_status_ = status;
    mode_ = Mode::kEnded;
  }

  void fail(const std::string & why) {end_run("FAILED: " + why);}

  [[nodiscard]] bool is_for_me(const Handshake & m) const {return m.drone_id == id_;}

  // D19 / D18.
  void on_mission(const Handshake & m)
  {
    if (!is_for_me(m)) {return;}
    if (mode_ != Mode::kParked && mode_ != Mode::kOnPad) {
      fail("MISSION_ASSIGN in " + std::string(name(mode_)));
      return;
    }
    if (m.code == "PARK") {
      if (spot_ != kSpotOut) {fail("PARK away from OUT needs a parking area"); return;}
      go(Mode::kParked, "D18");
      drone_status("PARKED");
      end_run("COMPLETE");
      return;
    }
    mission_ = m.code;
    dest_ = m.dest_pad;
    go(mode_, "D19");
    Handshake r = to_pad("DEPART_REQUEST", pad_);
    r.dest_pad = dest_;
    r.code = mission_;
    bus_.send(r);
  }

  // D01: takeoff to the cruise height over the spot it stands on.
  void on_depart_clear(const Handshake & m)
  {
    if (!is_for_me(m) || m.pad_id != pad_) {return;}
    if (mode_ != Mode::kParked && mode_ != Mode::kOnPad) {fail("DEPART_CLEAR while flying"); return;}
    go(Mode::kTakeoff, "D01");
    const Vec3 p = px4_.position();
    maneuver_.start({{Leg::Kind::kEngage, {p[0], p[1], -cruise_alt_}}}, yaw_);
  }

  // D04.
  void on_grant(const Handshake & m)
  {
    if (!is_for_me(m) || m.pad_id != dest_) {return;}
    if (mode_ != Mode::kHolding) {fail("LANDING_GRANT outside HOLDING"); return;}
    grant_spot_ = m.spot;
    go(Mode::kApproach, "D04");
    const PadGeo & p = pad(dest_);
    const Vec3 spot = ned(m.spot == kSpotIn ? p.in : p.out, cruise_alt_);
    const Vec3 above{spot[0], spot[1], -approach_alt_};
    begin_landing({{Leg::Kind::kLine, spot, 2.0}, {Leg::Kind::kVertical, above},
        {Leg::Kind::kHold, above}},
      m.spot == kSpotIn ? p.in_marker : p.out_marker);
  }

  void begin_landing(std::deque<Leg> legs, std::uint16_t marker)
  {
    landing_ = Landing::kFlyToSpot;
    landing_marker_ = marker;
    maneuver_.start(std::move(legs), yaw_);
  }

  // D15: release after release_s.
  void on_release_cmd(const Handshake & m)
  {
    if (!is_for_me(m) || m.pad_id != pad_ || mode_ != Mode::kOnPad || spot_ != kSpotIn) {return;}
    go(Mode::kOnPad, "D15");
    release_due_ = now() + rclcpp::Duration::from_seconds(release_s_);
  }

  // D15: hop IN -> OUT.
  void on_hop_cmd(const Handshake & m)
  {
    if (!is_for_me(m) || m.pad_id != pad_ || mode_ != Mode::kOnPad) {return;}
    go(Mode::kHopping, "D15");
    const PadGeo & p = pad(pad_);
    const Vec3 here = px4_.position();
    const Vec3 out = ned(p.out, hop_alt_);
    begin_landing({{Leg::Kind::kEngage, {here[0], here[1], -hop_alt_}},
        {Leg::Kind::kLine, out, hop_speed_}, {Leg::Kind::kHold, out}}, p.out_marker);
    grant_spot_ = kSpotOut;
  }

  // D15: latch after latch_s; the latch closes if the pod is inside its capture funnel.
  void on_latch_cmd(const Handshake & m)
  {
    if (!is_for_me(m) || m.pad_id != pad_ || mode_ != Mode::kOnPad || spot_ != kSpotOut) {return;}
    go(Mode::kOnPad, "D15");
    latch_pod_ = m.pod_id;
    latch_due_ = now() + rclcpp::Duration::from_seconds(latch_s_);
  }

  // D17.
  void on_service_complete(const Handshake & m)
  {
    if (!is_for_me(m) || m.pad_id != pad_ || mode_ != Mode::kOnPad) {return;}
    go(Mode::kOnPad, "D17");
    Handshake r = to_pad("MISSION_REQUEST", pad_);
    r.spot = spot_;
    r.pod_id = pod_;
    r.soc = px4_.soc();
    r.code = m.code;
    bus_.send(r);
  }

  void step()
  {
    if (mode_ == Mode::kEnded) {return;}
    const rclcpp::Time t = now();
    const double dt = last_step_.nanoseconds() == 0 ? 0.0 : (t - last_step_).seconds();
    last_step_ = t;
    if (!started_) {
      if (px4_.ready() && truth_.has("drone")) {
        started_ = true;
        yaw_ = px4_.heading();
        drone_status("PARKED");
      }
      return;
    }
    if (maneuver_.status() == Maneuver::Status::kFailed) {
      fail(maneuver_.failure());
      return;
    }
    maneuver_.step(dt);
    step_ground(t);
    step_flight(t);
  }

  void step_ground(const rclcpp::Time & t)
  {
    if (release_due_ && t >= *release_due_) {
      release_due_.reset();
      latch_pubs_["dd_pod_" + std::to_string(pod_) + "/detach"]->publish(std_msgs::msg::Empty());
      Handshake r = to_pad("RELEASE_REPORT", pad_);
      r.flag = true;
      bus_.send(r);
      pod_ = kNoId32;
      order_ = kNoId32;
    }
    if (latch_due_ && t >= *latch_due_) {
      latch_due_.reset();
      const std::string pod = "dd_pod_" + std::to_string(latch_pod_);
      const Vec3 me = truth_.enu("drone");
      const bool locked = truth_.horizontal_distance(pod, me[0], me[1]) <= capture_m_;
      if (locked) {
        latch_pubs_[pod + "/attach"]->publish(std_msgs::msg::Empty());
        pod_ = latch_pod_;
      }
      Handshake r = to_pad("LATCH_REPORT", pad_);
      r.pod_id = latch_pod_;
      r.flag = locked;
      bus_.send(r);
    }
  }

  void step_flight(const rclcpp::Time & t)
  {
    const bool leg_done = maneuver_.status() == Maneuver::Status::kDone;
    if (mode_ == Mode::kTakeoff && leg_done) {
      // D02: top of climb; the lane starts.
      go(Mode::kEnRoute, "D02");
      Handshake d = to_pad("DEPARTED", pad_);
      d.spot = spot_;
      bus_.send(d);
      pad_ = kNoId16;
      drone_status("EN_ROUTE");
      const Vec3 fix = ned(pad(dest_).center, cruise_alt_);
      std::deque<Leg> lane{{Leg::Kind::kLine, fix, cruise_speed_}, {Leg::Kind::kHold, fix}};
      if (vtol_) {
        // Fixed-wing cruise, back to multicopter before the fix, then the rest of the way.
        lane.push_front({Leg::Kind::kFixedWing, fix, cruise_speed_, back_transition_m_});
      }
      maneuver_.start(std::move(lane), yaw_);
    } else if (mode_ == Mode::kEnRoute && maneuver_.leg() == Leg::Kind::kHold) {
      // D03: at the holding fix of dest_pad.
      go(Mode::kHolding, "D03");
      Handshake r = to_pad("LANDING_REQUEST", dest_);
      r.pod_id = pod_;
      r.soc = px4_.soc();
      bus_.send(r);
    } else if (landing_ != Landing::kNone) {
      step_landing(t, leg_done);
    }
  }

  void step_landing(const rclcpp::Time & t, bool leg_done)
  {
    if (landing_ == Landing::kFlyToSpot && maneuver_.leg() == Leg::Kind::kHold) {
      awaited_marker_ = landing_marker_;
      detections_ = 0;
      marker_pub_->publish(std_msgs::msg::UInt16().set__data(landing_marker_));
      wait_since_ = t;
      landing_ = Landing::kWaitTarget;
    } else if (landing_ == Landing::kWaitTarget) {
      if (detections_ >= min_detections_) {
        landing_ = Landing::kLand;
        maneuver_.start({{Leg::Kind::kPrecland, {}}}, yaw_);
      } else if ((t - wait_since_).seconds() > target_timeout_s_) {
        fail("marker " + std::to_string(landing_marker_) + " not detected");
      }
    } else if (landing_ == Landing::kLand && leg_done) {
      landing_ = Landing::kNone;
      awaited_marker_ = 0xFFFF;
      landed();
    }
  }

  // D05 (approach) or D15 (hop done): on the ground at the granted spot.
  void landed()
  {
    if (mode_ == Mode::kApproach) {
      pad_ = dest_;
      go(Mode::kOnPad, "D05");
    } else {
      go(Mode::kOnPad, "D15");
    }
    spot_ = grant_spot_;
    Handshake l = to_pad("LANDED", pad_);
    l.spot = spot_;
    l.pod_id = pod_;
    l.soc = px4_.soc();
    bus_.send(l);
    if (spot_ == kSpotIn) {
      drone_status("ON_PAD");
    }
  }

  HandshakeBus bus_;
  Px4Link px4_;
  Maneuver maneuver_;
  TruthPoses truth_;
  std::vector<PadGeo> pads_;
  std::uint16_t id_{}, pad_{}, dest_ = kNoId16;
  std::uint8_t spot_ = kSpotOut, grant_spot_ = kNoCode;
  std::uint32_t pod_{}, order_{}, latch_pod_ = kNoId32;
  std::string mission_, ended_status_;
  Mode mode_ = Mode::kParked;
  Landing landing_ = Landing::kNone;
  std::uint16_t landing_marker_ = 0xFFFF, awaited_marker_ = 0xFFFF;
  std::uint32_t detections_ = 0, min_detections_{};
  double cruise_speed_{}, cruise_alt_{}, approach_alt_{}, hop_alt_{}, hop_speed_{};
  double release_s_{}, latch_s_{}, capture_m_{}, target_timeout_s_{}, yaw_ = 0.0;
  bool started_ = false, vtol_ = false;
  double back_transition_m_ = 0.0;
  std::optional<rclcpp::Time> release_due_, latch_due_;
  rclcpp::Time last_step_{0, 0, RCL_ROS_TIME}, wait_since_{0, 0, RCL_ROS_TIME};
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> latch_pubs_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<dd_interfaces::msg::TargetStatus>::SharedPtr detector_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dd_flight

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dd_flight::DroneAgent>());
  rclcpp::shutdown();
  return 0;
}
