#!/usr/bin/env bash
# Build the Phase 4 ROS 2 workspace (px4/ros2_ws): px4_msgs at the commit the PX4 v1.17.0 SITL
# build matches, plus this repo's packages. No root needed; needs ROS 2 Humble in /opt/ros and
# a built PX4 tree (for its generated MAVLink headers). Usage: px4/setup_ws.sh [colcon args]
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HERE/ros2_ws"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_MSGS_URL="https://github.com/PX4/px4_msgs.git"
# release/1.17, the commit quad-autonomy-sim verified against PX4 v1.17.0.
PX4_MSGS_COMMIT="86d8239e962f6939e05c3737784f60c02fa884db"
MAVLINK_INCLUDE="$PX4_DIR/build/px4_sitl_default/mavlink"

[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS 2 Humble not found in /opt/ros/humble" >&2; exit 1; }
[[ -f "$MAVLINK_INCLUDE/common/mavlink.h" ]] || { echo "PX4 SITL not built: cd $PX4_DIR && make px4_sitl" >&2; exit 1; }

msgs="$WS/src/external/px4_msgs"
if [[ ! -d "$msgs/.git" ]]; then
  git clone --quiet "$PX4_MSGS_URL" "$msgs"
fi
git -C "$msgs" fetch --quiet origin "$PX4_MSGS_COMMIT" 2>/dev/null || true
git -C "$msgs" checkout --quiet "$PX4_MSGS_COMMIT"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
cd "$WS"
# Always Release: an unoptimised build of image-processing nodes is far slower.
colcon build \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DDD_MAVLINK_INCLUDE="$MAVLINK_INCLUDE" "$@"
