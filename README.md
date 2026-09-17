1.13.3 px4:

source ~/PX4-v1.13.3/Tools/setup_gazebo.bash ~/PX4-v1.13.3 ~/PX4-v1.13.3/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-v1.13.3:~/PX4-v1.13.3/Tools/sitl_gazebo
source ~/livox_ws/devel/setup.bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/livox_ws/devel/lib
source /opt/ros/noetic/setup.bash
source ~/PX4-v1.13.3/Tools/setup_gazebo.bash ~/PX4-v1.13.3 ~/PX4-v1.13.3/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-v1.13.3
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-v1.13.3/Tools/sitl_gazebo

roslaunch px4 mavros_posix_sitl.launch
param set EKF2_HGT_MODE 3
aram set EKF2_AID_MASK 24
param set COM_RCL_EXCEPT 4