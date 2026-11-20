#!/bin/bash
# Запуск контейнера: /entrypoint.sh ping или /entrypoint.sh T2

if [ "$1" == "ping" ]; then
    # только один json в stdout
    echo '{"protocol_version": "1.0", "supported_tasks": ["T2"], "self_trained_rl": false, "preferred_speed_mps": null}'
    exit 0
fi

if [ "$1" == "T2" ]; then
    source /opt/ros/humble/setup.bash >&2
    source /home/ros/go1_ws/install/setup.bash >&2
    export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
    export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/ros_logs}
    mkdir -p $ROS_LOG_DIR 2>/dev/null
    exec python3 -u /opt/t2/safety/safety_node.py
fi

echo "unknown mode: '$1' (use ping or T2)" >&2
exit 2
