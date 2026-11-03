FROM tiryoh/ros2-desktop-vnc:jazzy

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rviz2 \
      ros-jazzy-ros-gz \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-robot-state-publisher \
      liburdfdom-tools \
      python3-numpy python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc \
 && echo "[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc
