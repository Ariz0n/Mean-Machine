FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*


ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ARG USERNAME=participant
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

WORKDIR /ros2_ws
COPY src/ ./src/
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && chown -R $USERNAME:$USERNAME /ros2_ws

USER $USERNAME

ENTRYPOINT ["/entrypoint.sh"]
