// Fleet-manager stub for one delivery: when the drone reports PARKED on A's OUT, assign the
// pickup waiting on B's OUT (PICKUP_ASSIGNED to B) and the delivery to B (MISSION_ASSIGN
// DELIVER); when the drone asks for its next mission at B, park it there (MISSION_ASSIGN PARK).
// The real policies are the Rust core's (docs/spec.md 6); this stub only drives the handshake.

#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "dd_flight/handshake.hpp"

namespace dd_flight
{

class FleetManagerStub : public rclcpp::Node
{
public:
  FleetManagerStub()
  : Node("fleet_manager"), bus_(*this, "FLEET_MANAGER")
  {
    drone_ = static_cast<std::uint16_t>(declare_parameter<int>("drone_id"));
    dest_ = static_cast<std::uint16_t>(declare_parameter<int>("dest_pad"));
    order_ = static_cast<std::uint32_t>(declare_parameter<int>("order_id"));
    pickup_pod_ = static_cast<std::uint32_t>(declare_parameter<int>("pickup_pod"));
    pickup_order_ = static_cast<std::uint32_t>(declare_parameter<int>("pickup_order_id"));
    bus_.on("DRONE_STATUS", [this](const Handshake & m) {
        if (m.drone_id == drone_ && m.code == "PARKED" && !assigned_) {
          assigned_ = true;
          Handshake p = blank("PICKUP_ASSIGNED");
          p.pad_id = dest_;
          p.pod_id = pickup_pod_;
          p.order_id = pickup_order_;
          p.drone_id = drone_;
          bus_.send(p);
          assign("DELIVER");
        }
      });
    bus_.on("MISSION_REQUEST", [this](const Handshake & m) {
        if (m.drone_id == drone_) {assign("PARK");}
      });
  }

private:
  void assign(const char * mission)
  {
    Handshake m = blank("MISSION_ASSIGN");
    m.drone_id = drone_;
    m.code = mission;
    m.dest_pad = dest_;
    m.order_id = order_;
    m.pickup_order_id = pickup_order_;
    bus_.send(m);
  }

  HandshakeBus bus_;
  std::uint16_t drone_{}, dest_{};
  std::uint32_t order_{}, pickup_pod_{}, pickup_order_{};
  bool assigned_ = false;
};

}  // namespace dd_flight

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dd_flight::FleetManagerStub>());
  rclcpp::shutdown();
  return 0;
}
