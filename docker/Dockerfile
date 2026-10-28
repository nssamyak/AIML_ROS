# ROS 2 Jazzy + Gazebo Harmonic with everything the HB bot stack needs.
# Useful for students on Windows/macOS and for headless testing.
# The workspace is mounted in at run time, so edits on the host are picked up.
#
#   docker build -t hb_bot:jazzy docker
#   docker run -it --rm -v "$PWD/ros2_ws/src:/ros2_ws/src" hb_bot:jazzy
#     -> inside:  cd /ros2_ws && colcon build && source install/setup.bash
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-ros-gz \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-slam-toolbox \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-rplidar-ros \
      python3-serial \
      python3-numpy \
      python3-pytest \
      python3-colcon-common-extensions \
      liburdfdom-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> /root/.bashrc
CMD ["bash"]
