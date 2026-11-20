# Образ для T2
# Берем образ симулятора организаторов: там уже есть Ubuntu 22.04, ROS 2 Humble, Cyclone DDS,
# сообщения unitree_go, MuJoCo 3.3.7 и модель Go1
FROM registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim@sha256:c79ddbece6cc7d00f114a9a9a09be0bae061ebb35ca722bcbe51bf75fce1b2e3

USER root

COPY safety/ /opt/t2/safety/
COPY entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh && chmod -R a+rX /opt/t2

ENV SCENE_FILE=/opt/mujoco_menagerie/unitree_go1/scene.xml \
    PAIRS_FILE=/opt/t2/safety/pairs.csv \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_LOG_DIR=/tmp/ros_logs \
    PYTHONDONTWRITEBYTECODE=1

# без root
USER ros
WORKDIR /tmp
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ping"]
