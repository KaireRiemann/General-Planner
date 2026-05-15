#!/bin/zsh
echo 'nv' | sudo -S chmod 777 /dev/tty* & sleep 1;
roslaunch mavros px4.launch & sleep 6;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;
source devel/setup.zsh;
roslaunch livox_ros_driver2 msg_MID360.launch & sleep 5;
roslaunch fast_lio mapping_mid360.launch & sleep 5;
roslaunch ekf_quat ekf_quat_lidar_mavros.launch & sleep 5;
roslaunch task_planner click_real.launch
# roslaunch diff_planner exp_rviz.launch & sleep 1;
wait;
