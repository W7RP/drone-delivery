// Pad controller: one pad's visit machine (docs/spec.md 3.3, 3.4) for the normal delivery visit.
//
// Rows implemented: V01 (LANDING_REQUEST, spot free -> APPROACH, LANDING_GRANT), V06 (LANDED IN
// with a pod -> RELEASING, RELEASE_CMD), V15 (RELEASE_REPORT -> VERIFY_RELEASE), V18 (release
// verified -> WAIT_OUT, POD_DELIVERED), V21 (OUT drone-free with the pickup pod -> HOPPING,
// HOP_CMD), V24 (LANDED OUT with the pickup pod -> LATCHING, LATCH_CMD), V28 (LATCH_REPORT ->
// VERIFY_LOCK), V32 (lock verified, no charger -> READY, POD_LATCHED, SERVICE_COMPLETE), V34
// (DEPART_REQUEST -> DEPARTING, DEPART_CLEAR), V36 (DEPARTED -> CLOSED), and P07
// (PICKUP_ASSIGNED). Other rows are failure branches (later phases): reaching one ends the run
// as FAILED.
//
// Sensors (spec A12): the spot presence and pod sensors read Gazebo ground truth through
// TruthPoses. Charging is not simulated in this phase: the pad reports no charger (A58).

#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "dd_flight/handshake.hpp"
#include "dd_flight/sim_sensors.hpp"

namespace dd_flight
{

struct Visit
{
  std::string state;
  std::uint8_t spot = kNoCode;
  std::uint32_t pod = kNoId32;     // pod the drone landed with
  float soc = std::nanf("");
  bool latch_open = false, locked = false;
};

class PadController : public rclcpp::Node
{
public:
  PadController()
  : Node("pad_controller"), bus_(*this, "PAD"), truth_(*this, {"dd_quad", "dd_pod_1", "dd_pod_2"})
  {
    id_ = static_cast<std::uint16_t>(declare_parameter<int>("pad_id"));
    const auto in = declare_parameter<std::vector<double>>("in_enu_m");
    const auto out = declare_parameter<std::vector<double>>("out_enu_m");
    in_ = {in[0], in[1], 0};
    out_ = {out[0], out[1], 0};
    spot_radius_ = declare_parameter<double>("spot_sensor_radius_m");
    pod_radius_ = declare_parameter<double>("in_pod_sensor_radius_m");
    pod_rest_z_ = declare_parameter<double>("pod_rest_max_z_m");
    capture_m_ = declare_parameter<double>("latch_capture_radius_m");
    release_verify_s_ = declare_parameter<double>("release_verify_s");
    latch_verify_s_ = declare_parameter<double>("latch_verify_s");
    const auto out_pod = declare_parameter<int>("out_pod");
    out_pod_ = out_pod < 0 ? kNoId32 : static_cast<std::uint32_t>(out_pod);
    const auto parked = declare_parameter<int>("parked_drone_on_out");

    bus_.on("LANDING_REQUEST", [this](const Handshake & m) {if (mine(m)) {on_landing_request(m);}});
    bus_.on("LANDED", [this](const Handshake & m) {if (mine(m)) {on_landed(m);}});
    bus_.on("RELEASE_REPORT", [this](const Handshake & m) {if (mine(m)) {on_release_report(m);}});
    bus_.on("LATCH_REPORT", [this](const Handshake & m) {if (mine(m)) {on_latch_report(m);}});
    bus_.on("DEPART_REQUEST", [this](const Handshake & m) {if (mine(m)) {on_depart_request(m);}});
    bus_.on("DEPARTED", [this](const Handshake & m) {if (mine(m)) {on_departed(m);}});
    bus_.on("PICKUP_ASSIGNED", [this](const Handshake & m) {if (mine(m)) {on_pickup(m);}});
    status_pub_ = create_publisher<std_msgs::msg::String>("/dd/run_status",
        rclcpp::QoS(1).reliable().transient_local());

    if (parked >= 0) {
      // Spec 3.3: a drone placed on OUT at t = 0 starts with a visit in READY.
      const auto d = static_cast<std::uint16_t>(parked);
      out_drone_ = d;
      visits_[d] = Visit{"READY", kSpotOut};
      init_drone_ = d;
    }
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(50),
        [this]() {step();});
  }

private:
  [[nodiscard]] bool mine(const Handshake & m) const {return m.pad_id == id_;}

  void go(std::uint16_t d, const std::string & row, const std::string & to)
  {
    auto it = visits_.find(d);
    const std::string from = it == visits_.end() ? "(none)" : it->second.state;
    bus_.transition("VISIT", row, id_, d, from, to);
    visits_[d].state = to;
  }

  void fail(const std::string & why)
  {
    RCLCPP_ERROR(get_logger(), "%s", why.c_str());
    status_pub_->publish(std_msgs::msg::String().set__data("FAILED: pad " + std::to_string(id_) +
        ": " + why));
  }

  [[nodiscard]] Handshake to_drone(const std::string & msg_name, std::uint16_t d) const
  {
    Handshake m = blank(msg_name);
    m.pad_id = id_;
    m.drone_id = d;
    return m;
  }

  [[nodiscard]] bool drone_on(const Vec3 & spot) const
  {
    const Vec3 p = truth_.enu("dd_quad");
    return truth_.horizontal_distance("dd_quad", spot[0], spot[1]) <= spot_radius_ && p[2] < 0.5;
  }

  // A pod resting on IN (not hanging in a latch): close to the spot and down on the slab.
  [[nodiscard]] bool pod_on_in(std::uint32_t pod) const
  {
    const std::string name = "dd_pod_" + std::to_string(pod);
    return truth_.has(name) && truth_.horizontal_distance(name, in_[0], in_[1]) <= pod_radius_ &&
           truth_.enu(name)[2] <= pod_rest_z_;
  }

  // V01 (normal path: IN free for a drone with a pod).
  void on_landing_request(const Handshake & m)
  {
    if (visits_.contains(m.drone_id)) {fail("LANDING_REQUEST with an open visit"); return;}
    const bool in_free = in_drone_ == kNoId16 && in_pod_ == kNoId32 && !drone_on(in_);
    if (m.pod_id == kNoId32 || !in_free) {fail("normal path needs IN free for a loaded drone"); return;}
    go(m.drone_id, "V01", "APPROACH");
    Visit & v = visits_[m.drone_id];
    v.spot = kSpotIn;
    in_reserved_ = m.drone_id;
    Handshake g = to_drone("LANDING_GRANT", m.drone_id);
    g.spot = kSpotIn;
    bus_.send(g);
  }

  void on_landed(const Handshake & m)
  {
    auto it = visits_.find(m.drone_id);
    if (it == visits_.end()) {fail("LANDED without a visit"); return;}
    Visit & v = it->second;
    v.soc = m.soc;
    if (v.state == "APPROACH" && m.spot == kSpotIn && m.pod_id != kNoId32) {
      // V06.
      go(m.drone_id, "V06", "RELEASING");
      in_drone_ = m.drone_id;
      in_reserved_ = kNoId16;
      v.pod = m.pod_id;
      Handshake c = to_drone("RELEASE_CMD", m.drone_id);
      c.attempt = 1;
      bus_.send(c);
    } else if (v.state == "HOPPING" && m.spot == kSpotOut && out_pod_ != kNoId32 &&
      out_pod_ == pickup_pod_)
    {
      // V24.
      go(m.drone_id, "V24", "LATCHING");
      in_drone_ = kNoId16;
      out_drone_ = m.drone_id;
      out_reserved_ = kNoId16;
      Handshake c = to_drone("LATCH_CMD", m.drone_id);
      c.pod_id = out_pod_;
      c.attempt = 1;
      bus_.send(c);
    } else {
      fail("LANDED in " + v.state + " is not on the normal path");
    }
  }

  // V15.
  void on_release_report(const Handshake & m)
  {
    auto it = visits_.find(m.drone_id);
    if (it == visits_.end() || it->second.state != "RELEASING") {fail("unexpected RELEASE_REPORT"); return;}
    it->second.latch_open = m.flag;
    go(m.drone_id, "V15", "VERIFY_RELEASE");
    verify_due_ = now() + rclcpp::Duration::from_seconds(release_verify_s_);
    verify_drone_ = m.drone_id;
  }

  // V28.
  void on_latch_report(const Handshake & m)
  {
    auto it = visits_.find(m.drone_id);
    if (it == visits_.end() || it->second.state != "LATCHING") {fail("unexpected LATCH_REPORT"); return;}
    it->second.locked = m.flag;
    go(m.drone_id, "V28", "VERIFY_LOCK");
    verify_due_ = now() + rclcpp::Duration::from_seconds(latch_verify_s_);
    verify_drone_ = m.drone_id;
  }

  // V34.
  void on_depart_request(const Handshake & m)
  {
    auto it = visits_.find(m.drone_id);
    if (it == visits_.end() || it->second.state != "READY") {fail("DEPART_REQUEST outside READY"); return;}
    go(m.drone_id, "V34", "DEPARTING");
    bus_.send(to_drone("DEPART_CLEAR", m.drone_id));
  }

  // V36.
  void on_departed(const Handshake & m)
  {
    auto it = visits_.find(m.drone_id);
    if (it == visits_.end() || it->second.state != "DEPARTING") {fail("unexpected DEPARTED"); return;}
    go(m.drone_id, "V36", "CLOSED");
    if (it->second.spot == kSpotOut) {out_drone_ = kNoId16;}
    if (it->second.spot == kSpotIn) {in_drone_ = kNoId16;}
    visits_.erase(it);
  }

  // P07: the pickup for the drone's next visit.
  void on_pickup(const Handshake & m)
  {
    if (m.pod_id != out_pod_) {fail("PICKUP_ASSIGNED for a pod not on OUT"); return;}
    pickup_pod_ = m.pod_id;
    pickup_drone_ = m.drone_id;
    bus_.transition("PAD", "P07", id_, m.drone_id, "-", "pickup pod " + std::to_string(m.pod_id));
  }

  void step()
  {
    if (init_drone_) {
      bus_.transition("VISIT", "INIT", id_, *init_drone_, "(none)", "READY");
      init_drone_.reset();
    }
    if (!verify_due_ || now() < *verify_due_) {return;}
    verify_due_.reset();
    Visit & v = visits_[verify_drone_];
    if (v.state == "VERIFY_RELEASE") {
      if (!(v.latch_open && pod_on_in(v.pod))) {fail("release not verified (V19/V20)"); return;}
      // V18: delivery measurement point.
      go(verify_drone_, "V18", "WAIT_OUT");
      in_pod_ = v.pod;
      Handshake d = blank("POD_DELIVERED");
      d.pad_id = id_;
      d.pod_id = v.pod;
      d.drone_id = verify_drone_;
      bus_.send(d);
      // V21, evaluated on WAIT_OUT entry.
      const bool out_free = out_drone_ == kNoId16 && out_reserved_ == kNoId16 && !drone_on(out_);
      if (!(out_free && out_pod_ != kNoId32 && out_pod_ == pickup_pod_ &&
        pickup_drone_ == verify_drone_))
      {
        fail("OUT not ready with the pickup pod (V21 guard)");
        return;
      }
      go(verify_drone_, "V21", "HOPPING");
      out_reserved_ = verify_drone_;
      bus_.send(to_drone("HOP_CMD", verify_drone_));
    } else if (v.state == "VERIFY_LOCK") {
      const std::string pod = "dd_pod_" + std::to_string(out_pod_);
      const Vec3 d = truth_.enu("dd_quad");
      if (!(v.locked && truth_.horizontal_distance(pod, d[0], d[1]) <= capture_m_)) {
        fail("lock not verified (V29/V30)");
        return;
      }
      // V32: no charger in this phase, so READY.
      go(verify_drone_, "V32", "READY");
      const std::uint32_t latched = out_pod_;
      out_pod_ = kNoId32;
      pickup_pod_ = kNoId32;
      Handshake l = blank("POD_LATCHED");
      l.pad_id = id_;
      l.pod_id = latched;
      l.drone_id = verify_drone_;
      bus_.send(l);
      Handshake c = to_drone("SERVICE_COMPLETE", verify_drone_);
      c.code = "NORMAL";
      c.spot = kSpotOut;
      c.pod_id = latched;
      c.soc = v.soc;
      bus_.send(c);
    }
  }

  HandshakeBus bus_;
  TruthPoses truth_;
  std::uint16_t id_{};
  Vec3 in_{}, out_{};
  double spot_radius_{}, pod_radius_{}, pod_rest_z_{}, capture_m_{};
  double release_verify_s_{}, latch_verify_s_{};
  std::map<std::uint16_t, Visit> visits_;
  std::uint16_t in_drone_ = kNoId16, out_drone_ = kNoId16, in_reserved_ = kNoId16,
    out_reserved_ = kNoId16, pickup_drone_ = kNoId16, verify_drone_ = kNoId16;
  std::uint32_t in_pod_ = kNoId32, out_pod_ = kNoId32, pickup_pod_ = kNoId32;
  std::optional<std::uint16_t> init_drone_;
  std::optional<rclcpp::Time> verify_due_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dd_flight

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dd_flight::PadController>());
  rclcpp::shutdown();
  return 0;
}
