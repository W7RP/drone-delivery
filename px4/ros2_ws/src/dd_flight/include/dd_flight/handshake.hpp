// The spec 4.2 handshake over ROS 2: one topic per message name (/dd/msg/<lower-case name>),
// one message type (dd_interfaces/Handshake), per-sender sequence numbers (spec 4.1), and the
// spec's state transitions on /dd/transitions. Shared by the drone agent, pad controllers and
// the fleet-manager stub. Sim time comes from the node clock (use_sim_time: Gazebo's clock).
#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "dd_interfaces/msg/handshake.hpp"
#include "dd_interfaces/msg/transition.hpp"

namespace dd_flight
{

using dd_interfaces::msg::Handshake;
using dd_interfaces::msg::Transition;

constexpr std::uint16_t kNoId16 = Handshake::NONE_ID16;
constexpr std::uint32_t kNoId32 = Handshake::NONE_ID32;
constexpr std::uint8_t kNoCode = Handshake::NONE_CODE;
constexpr std::uint8_t kSpotIn = 0;
constexpr std::uint8_t kSpotOut = 1;

[[nodiscard]] inline std::string topic_for(const std::string & name)
{
  std::string t = "/dd/msg/";
  for (char c : name) {
    t += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return t;
}

[[nodiscard]] inline std::uint64_t sim_us(rclcpp::Node & node)
{
  return static_cast<std::uint64_t>(node.now().nanoseconds() / 1000);
}

// A message with every field at its NONE value.
[[nodiscard]] inline Handshake blank(const std::string & name)
{
  Handshake m;
  m.name = name;
  m.drone_id = kNoId16;
  m.pad_id = kNoId16;
  m.pod_id = kNoId32;
  m.order_id = kNoId32;
  m.pickup_order_id = kNoId32;
  m.dest_pad = kNoId16;
  m.spot = kNoCode;
  m.soc = std::numeric_limits<float>::quiet_NaN();
  m.attempt = 0;
  m.flag = false;
  m.dwell_s = std::numeric_limits<float>::quiet_NaN();
  m.queue_pos = 0;
  return m;
}

class HandshakeBus
{
public:
  using Handler = std::function<void(const Handshake &)>;

  // `sender` is DRONE, PAD or FLEET_MANAGER.
  HandshakeBus(rclcpp::Node & node, std::string sender)
  : node_(node), sender_(std::move(sender)),
    transition_pub_(node.create_publisher<Transition>("/dd/transitions", qos()))
  {}

  // Lossless and in order: the normal path has no message loss (handshake.p_msg_loss = 0).
  [[nodiscard]] static rclcpp::QoS qos() {return rclcpp::QoS(100).reliable();}

  void send(Handshake m)
  {
    m.msg_seq = ++seq_;
    m.sent_t_us = sim_us(node_);
    m.sender = sender_;
    auto & pub = publishers_[m.name];
    if (!pub) {
      pub = node_.create_publisher<Handshake>(topic_for(m.name), qos());
    }
    pub->publish(m);
  }

  void on(const std::string & name, Handler handler)
  {
    subscriptions_.push_back(node_.create_subscription<Handshake>(
        topic_for(name), qos(), [handler](const Handshake & m) {handler(m);}));
  }

  void transition(
    const std::string & machine, const std::string & row, std::uint16_t pad, std::uint16_t drone,
    const std::string & from, const std::string & to)
  {
    Transition t;
    t.t_us = sim_us(node_);
    t.machine = machine;
    t.row = row;
    t.pad_id = pad;
    t.drone_id = drone;
    t.from_state = from;
    t.to_state = to;
    transition_pub_->publish(t);
    RCLCPP_INFO(node_.get_logger(), "%s %s: %s -> %s", machine.c_str(), row.c_str(), from.c_str(),
      to.c_str());
  }

private:
  rclcpp::Node & node_;
  std::string sender_;
  std::uint32_t seq_ = 0;
  std::map<std::string, rclcpp::Publisher<Handshake>::SharedPtr> publishers_;
  std::vector<rclcpp::Subscription<Handshake>::SharedPtr> subscriptions_;
  rclcpp::Publisher<Transition>::SharedPtr transition_pub_;
};

}  // namespace dd_flight
