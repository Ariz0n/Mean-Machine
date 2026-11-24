# Образ для T4
# Берем образ симулятора организаторов: там уже есть Ubuntu 22.04, ROS 2 Humble, Cyclone DDS,
# numpy, scipy и tf2_ros
FROM registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim@sha256:c79ddbece6cc7d00f114a9a9a09be0bae061ebb35ca722bcbe51bf75fce1b2e3

USER root

COPY odom/ /opt/t4/odom/
COPY entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh && chmod -R a+rX /opt/t4 \
    && bash -c ". /opt/ros/humble/setup.bash && python3 -c 'import numpy, scipy.spatial, tf2_ros'"

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_LOG_DIR=/tmp/ros_logs \
    PYTHONDONTWRITEBYTECODE=1

# без root
USER ros
WORKDIR /tmp
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ping"]
