# Образ для T4 и T5
# Берем образ симулятора организаторов: там уже есть Ubuntu 22.04, ROS 2 Humble, Cyclone DDS,
# numpy, scipy, OpenCV (с ArUco) и tf2_ros. Для T5 доставляем Nav2.
FROM registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim@sha256:c79ddbece6cc7d00f114a9a9a09be0bae061ebb35ca722bcbe51bf75fce1b2e3

USER root

# Nav2 (ставится при сборке; во время испытания интернет не нужен)
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-navigation2 \
    && rm -rf /var/lib/apt/lists/*

COPY odom/ /opt/go1/odom/
COPY nav/ /opt/go1/nav/
COPY entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh /opt/go1/odom/odom_node.py /opt/go1/nav/map_odom.py /opt/go1/nav/aruco_node.py \
    && chmod -R a+rX /opt/go1 \
    && bash -c ". /opt/ros/humble/setup.bash && python3 -c 'import numpy, scipy.spatial, tf2_ros, cv2.aruco' \
                && for p in nav2_bt_navigator nav2_controller nav2_planner nav2_behaviors nav2_lifecycle_manager \
                   nav2_regulated_pure_pursuit_controller nav2_navfn_planner; do ros2 pkg prefix \$p || exit 1; done"

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_LOG_DIR=/tmp/ros_logs \
    PYTHONDONTWRITEBYTECODE=1

# без root
USER ros
WORKDIR /tmp
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ping"]
