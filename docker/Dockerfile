FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-vision-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-venv \
    python3-pip \
    git vim less curl \
 && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV GZ_SIM_RESOURCE_PATH=/ws/src/indoor_bot_gazebo/models:/ws/src/indoor_bot_gazebo/worlds

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc && \
    echo "[ -f /ws/.venv/bin/activate ] && source /ws/.venv/bin/activate" >> /root/.bashrc

WORKDIR /ws
CMD ["/bin/bash"]