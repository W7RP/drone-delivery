// Topic names for the PX4 uXRCE-DDS bridge. Copied from quad-autonomy-sim
// (ros2_ws/src/quad_offboard/include/quad_offboard/px4_topics.hpp, MIT).
//
// Since PX4 v1.16 the client appends "_v<N>" to a topic when its message has a non-zero
// MESSAGE_VERSION (e.g. /fmu/out/vehicle_status_v1). Building names from the px4_msgs
// constant keeps a node in lockstep with the px4_msgs it was compiled against.
#pragma once

#include <string>

namespace dd_flight
{

template<typename MsgT>
[[nodiscard]] std::string px4_topic(const std::string & base)
{
  std::string name = base;
  if constexpr (requires {MsgT::MESSAGE_VERSION;}) {
    if (MsgT::MESSAGE_VERSION != 0U) {
      name += "_v" + std::to_string(MsgT::MESSAGE_VERSION);
    }
  }
  return name;
}

}  // namespace dd_flight
