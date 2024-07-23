# Seeed Studio reComputer J401 Nvidia Jetson CSI camera launcher for ROS

This ROS package enables the use of multiple CSI cameras on the Seeed Studio reComputer and visualizes the output in rviz. It supports the reComputer J40/J30 series products, including Nvidia Jetson Orin NX and Orin Nano. The testing environment is Jetpack 5.x, Ubuntu 20.04 and ROS Noetic.

## Getting Start
### Installation
**Step 1**: Open a terminal on your reComputer, and Create a directory for your workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```
**Step 2**: Clone the desired ROS package 
```bash
git clone https://github.com/ZhuYaoHui1998/csi_camera_reader.git
```

**Step 3**:Build the workspace
```bash
cd ~/catkin_ws/
catkin_make
```
**Step 4**: Source the setup file again to update your environment
```bash
source devel/setup.bash
```

### Usage
To publish the one camera stream to the ROS topic /csi_cam_0/image_raw, use this command in the terminal:
```bash
roslaunch csi_camera_reader csi_camera.launch sensor_id:=0
```
You can also change `sensor_id` to 1 to match your interface.
```bash
roslaunch csi_camera_reader csi_camera.launch sensor_id:=1
```
If you want to open two CSI cameras simultaneously and publish topics to ROS, you can execute the following commands:
```bash
roslaunch csi_camera_reader dual_camera.launch
```
