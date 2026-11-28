#!/bin/bash
# Запуск контейнера: /entrypoint.sh ping | T4 | T5

if [ "$1" == "ping" ]; then
    # только один json в stdout
    echo '{"protocol_version": "1.0", "supported_tasks": ["T4", "T5"], "self_trained_rl": false, "preferred_speed_mps": null}'
    exit 0
fi

if [ "$1" == "T4" ]; then
    CMD="python3 -u /opt/go1/odom/odom_node.py"                # одометрия: /odom и TF odom -> trunk
elif [ "$1" == "T5" ]; then
    # одометрия + map -> odom + Nav2 + ArUco; при остановке ждем узлы не дольше 2 + 2 с
    CMD="ros2 launch /opt/go1/nav/t5.launch.py sigterm_timeout:=2 sigkill_timeout:=2"
else
    echo "unknown mode: '$1' (use ping, T4 or T5)" >&2
    exit 2
fi

source /opt/ros/humble/setup.bash >&2
source /home/ros/go1_ws/install/setup.bash >&2
# переменные ROS_DOMAIN_ID, RMW_IMPLEMENTATION и CYCLONEDDS_URI от запускателя не трогаем
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/ros_logs}
export ROS_HOME=${ROS_HOME:-/tmp/ros_home}
mkdir -p $ROS_LOG_DIR $ROS_HOME 2>/dev/null

# по ТЗ узлы должны работать до конца испытания: если упали - запускаем снова.
# контейнер останавливает инфраструктура (SIGTERM) - тогда останавливаем узлы и выходим.
STOP=0
PID=0
# (SIGINT фоновому процессу из скрипта не доходит - bash его игнорирует, поэтому шлем SIGTERM)
trap 'STOP=1; [ $PID -ne 0 ] && kill -TERM $PID 2>/dev/null' TERM INT
while true; do
    $CMD &
    PID=$!
    wait $PID
    CODE=$?
    if [ $STOP -eq 1 ]; then
        wait $PID 2>/dev/null
        exit 0
    fi
    echo "$1 exited with code $CODE, restarting in 1 s" >&2
    sleep 1
done
