ARG ROS_DISTRO=jazzy

FROM osrf/ros:${ROS_DISTRO}-simulation
LABEL authors="Joshua J. Damanik <joshuajdmk@gmail.com>"
LABEL version="0.0.1"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
  build-essential \
  python3-pip \
  cmake \
  wget \
  curl \
  && rm -rf /var/lib/apt/lists/*

# Install ROS2 dependencies
RUN apt-get update && apt-get install -y \
  ros-${ROS_DISTRO}-sdformat-urdf \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-joint-state-publisher \
  ros-${ROS_DISTRO}-robot-localization \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

COPY src src

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
  && colcon build --symlink-install \
  && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]