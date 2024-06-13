# ORB_SLAM3_ROS2
Forked from [ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2)</br>
Refer to orginal for installation instructions 

## System 
1. ROS Foxy
2. Ubuntu 20.04
3. D435i

## Commands 
```bash
## source ros
source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/local_setup.bash # workspace

## realsense node
ros2 run realsense2_camera realsense2_camera_node --ros-args -p unite_imu_method:=2

## orbslam node z
ros2 run orbslam3 stereo-inertial orbslam3_ros2/vocabulary/ORBvoc.txt orbslam3_ros2/config/stereo-inertial/RealSense_D435i.yaml false true
# rectification is not needed for d435i
# should publish topic of /camera_pose 

## build 
cd ~/colcon_ws
colcon build --symlink-install --packages-select orbslam3
```



