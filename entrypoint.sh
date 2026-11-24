#!/bin/bash
# Запуск контейнера: /entrypoint.sh ping | T4

if [ "$1" == "ping" ]; then
    # только один json в stdout
    echo '{"protocol_version": "1.0", "supported_tasks": ["T4"], "self_trained_rl": false, "preferred_speed_mps": null}'
    exit 0
fi

if [ "$1" != "T4" ]; then
    echo "unknown mode: '$1' (use ping or T4)" >&2
    exit 2
fi

source /opt/ros/humble/setup.bash >&2
source /home/ros/go1_ws/install/setup.bash >&2
# переменные ROS_DOMAIN_ID, RMW_IMPLEMENTATION и CYCLONEDDS_URI от запускателя не трогаем
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/ros_logs}
mkdir -p $ROS_LOG_DIR 2>/dev/null

# по ТЗ узел должен работать до конца испытания: если он упал - запускаем снова.
# контейнер останавливает инфраструктура (SIGTERM) - тогда останавливаем узел и выходим.
STOP=0
PID=0
trap 'STOP=1; [ $PID -ne 0 ] && kill -TERM $PID 2>/dev/null' TERM INT
while true; do
    python3 -u /opt/t4/odom/odom_node.py &
    PID=$!
    wait $PID
    CODE=$?
    if [ $STOP -eq 1 ]; then
        wait $PID 2>/dev/null
        exit 0
    fi
    echo "node exited with code $CODE, restarting in 1 s" >&2
    sleep 1
done
