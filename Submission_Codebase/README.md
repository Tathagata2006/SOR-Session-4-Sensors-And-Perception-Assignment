# Session 4 Assignment Submission

**Name:** Tathagata Roy <br>
**Roll No.:** 25B3954

## Overview

This project implements a mobile robot in ROS2 Jazzy and Gazebo with the following capabilities:

* RGBD Camera Integration
* 2D LiDAR
* 3D LiDAR
* IMU Integration
* EKF-based Sensor Fusion
* OpenCV Image Processing
* YOLOv8 Object Detection
* Autonomous Object Seeking and Navigation

---

# Folder Structure

## Main Simulation Package

```text
src/erc_gazebo_sensors/
```

Contains robot description, sensors, launch files, world files, and EKF configuration.

### Important Files

#### Robot Model

```text
src/erc_gazebo_sensors/urdf/erc_bot.urdf
```

Contains robot links, joints, camera, lidar, and IMU mounting.

#### Gazebo Sensor Configuration

```text
src/erc_gazebo_sensors/urdf/erc_bot.gazebo
```

Contains RGBD camera, 2D/3D lidar, and IMU sensor plugins.

#### Launch File

```text
src/erc_gazebo_sensors/launch/spawn_robot.launch.py
```

Launches Gazebo, RViz, bridges, EKF, and robot model.

#### EKF Configuration

```text
src/erc_gazebo_sensors/config/ekf.yaml
```

Configuration used for odometry and IMU sensor fusion.

---

# Python Nodes

```text
src/erc_gazebo_sensors_py/erc_gazebo_sensors_py/
```

Contains perception and navigation nodes.

### OpenCV Ball Tracking

```text
chase_the_ball.py
```

Implements color-based object detection and tracking using OpenCV.

### YOLOv8 Object Seeking

```text
yolo_detection_node.py
```

Implements:

* User target selection
* Search behaviour
* YOLOv8 object detection
* RGBD depth estimation
* Target tracking
* Autonomous navigation
* Mission completion logic

---

# Assignment Stage Mapping

## Stage 1 – Target Selection

File:

```text
yolo_detection_node.py
```

User enters target object at runtime.

---

## Stage 2 – Distance Estimation

File:

```text
yolo_detection_node.py
```

Distance obtained from RGBD depth image.

---

## Stage 3 – Search Behaviour

File:

```text
yolo_detection_node.py
```

Robot rotates until target appears.

---

## Stage 4 – Target Detection and Tracking

File:

```text
yolo_detection_node.py
```

YOLOv8 detection and camera-centering behaviour.

---

## Stage 5 – Autonomous Navigation

File:

```text
yolo_detection_node.py
```

Robot approaches the detected target.

---

## Stage 6 – Mission Completion

File:

```text
yolo_detection_node.py
```

Robot stops when target distance reaches the selected threshold.

---

# Running the Project

Launch simulation:

```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

Run object-seeking node:

```bash
ros2 run erc_gazebo_sensors_py yolo_detection_node
```

Enter a target object (for example: person, bottle, chair) when prompted.

---


