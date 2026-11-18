#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash >&2
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash >&2
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

case "$1" in
  ping)
    cat << 'JSON'
{
  "protocol_version": "1.0",
  "supported_tasks": ["T2", "T3", "T4", "T5"],
  "self_trained_rl": false,
  "preferred_speed_mps": 0.3
}
JSON
    exit 0
    ;;
  T2|T3|T4|T5)
    echo "Launching $1..." >&2
    sleep infinity
    ;;
  *)
    echo "Unknown argument: '$1'" >&2
    exit 1
    ;;
esac
