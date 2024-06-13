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

## realsense nodes
(specific for our usecase) 
ros2 launch krang_realsense_nodes launch_multiple_cameras.py
should publish topics with <camera_name>/ 


## orbslam node
### stereo-inertial (additional imu calibration is required, as of now its out of sync)
ros2 run orbslam3 stereo-inertial orbslam3_ros2/vocabulary/ORBvoc.txt orbslam3_ros2/config/stereo-inertial/RealSense_D435i.yaml <camera_name>

### stereo (working fine)
ros2 run orbslam3 stereo orbslam3_ros2/vocabulary/ORBvoc.txt orbslam3_ros2/config/stereo-inertial/RealSense_D435i.yaml <camera_name>

## build 
cd ~/colcon_ws
colcon build --symlink-install --packages-select orbslam3
```



