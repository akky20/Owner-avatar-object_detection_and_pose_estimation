# Object Detection and 3D Pose Estimation (ROS 2 Humble)

This repository provides a ROS 2 Humble implementation for real-time object detection and 3D pose estimation using the find\_object\_2d package and an Intel RealSense D435i RGB-D camera.

🎯 This project is a ROS 2 port of the original ROS 1 work done by [bandasaikrishna](https://github.com/bandasaikrishna/object_detection_and_3d_pose_estimation).
🔗 Original ROS 1 Repository: [https://github.com/bandasaikrishna/object\_detection\_and\_3d\_pose\_estimation](https://github.com/bandasaikrishna/object_detection_and_3d_pose_estimation)

✅ All functionality has been tested and validated in ROS 2 Humble using an Intel RealSense D435i camera.

---

## ✨ Features

* Object detection using find\_object\_2d
* 3D pose estimation with point cloud data
* Integrated with Intel RealSense D435i
* Clean ROS 2 node structure and launch files
* Real-time visualization in RViz

---

## 🧰 Dependencies

Make sure the following packages are installed:

* ROS 2 Humble Hawksbill
* Intel RealSense ROS 2 wrapper:

  * [https://github.com/IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros)
* find\_object\_2d (ported to ROS 2 manually or using ros1\_bridge)
* OpenCV
* PCL (Point Cloud Library)

---

## 🔧 Installation

1. Clone this repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/akky20/ros2_object_detection_3d_pose.git](https://github.com/akky20/object_detection_and_pose_estimation.git
```

2. Install dependencies:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:

```bash
colcon build
source install/setup.bash
```

---

## 📷 Launching the System

1. Launch the object detection node:

```bash
ros2 launch pointcloud_object_detection pointcloud_object_detection.launch.py
```

This will start:

* RGB-D image streaming
* find\_object\_2d for detection
* Point cloud-based pose estimation
* RViz with visual markers


---


## 📌 Notes

* This package was tested on Ubuntu 22.04 with ROS 2 Humble and a RealSense D435i camera.
* Ensure your camera is properly calibrated and the intrinsics are correctly published for accurate pose estimation.
* For best performance, use well-lit environments and objects with unique textures.
