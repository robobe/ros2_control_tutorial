FROM jazzy:allision

RUN apt-key del F42ED6FBAB17C654
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-xacro \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    python3-colcon-clean \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-plotjuggler-ros \
  && rm -rf /var/lib/apt/lists/*


  RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-tf-transformations \
  && rm -rf /var/lib/apt/lists/*