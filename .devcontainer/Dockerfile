FROM jazzy:allision

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-xacro \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros-gz-bridge \
  && rm -rf /var/lib/apt/lists/*


  RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-tf-transformations \
  && rm -rf /var/lib/apt/lists/*