export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/cyclonedds.xml
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source aliases.sh
export PS1="🐢 [\u@\h \W] \$ "
echo "hello ROS"