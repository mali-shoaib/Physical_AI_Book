---
title: "Ch3: VSLAM with Isaac ROS"
description: "Implement GPU-accelerated Visual SLAM using NVIDIA Isaac ROS on a simulated humanoid robot. Learn to build maps, localize, and visualize trajectories in real-time."
sidebar_position: 3
---

# Chapter 3: VSLAM with Isaac ROS

## Introduction

Visual Simultaneous Localization and Mapping (VSLAM) enables robots to build maps and localize themselves. This chapter covers GPU-accelerated VSLAM using NVIDIA Isaac ROS.

### Learning Objectives

1. Understand VSLAM fundamentals
2. Configure Isaac ROS Visual SLAM for stereo cameras
3. Run VSLAM in Isaac Sim with real-time visualization
4. Teleoperate the humanoid to build maps
5. Visualize VSLAM output in RViz2
6. Save and reload maps
7. Benchmark GPU vs CPU performance

### Prerequisites

- Chapters 1-2 completed
- ROS 2 Humble installed
- Understanding of TF frames

### Estimated Time

2-3 hours

---

## 3.1 VSLAM Fundamentals

VSLAM solves two problems simultaneously:
1. **Localization**: Where is the robot?
2. **Mapping**: What does the environment look like?

### VSLAM Pipeline

1. Feature Extraction - Detect ORB features
2. Feature Tracking - Match features across frames
3. Motion Estimation - Compute robot motion
4. Mapping - Build 3D map
5. Loop Closure - Correct drift

---

## 3.2 Isaac ROS Visual SLAM

### Why Isaac ROS?

- 10x faster than CPU SLAM
- Real-time at 30+ FPS on RTX GPUs
- Production-ready from NVIDIA
- ROS 2 native integration

### Installation

```bash
sudo apt-get install ros-humble-isaac-ros-visual-slam
```

---

## 3.3 Configuration

Camera calibration parameters:

```yaml
camera:
  left:
    topic: /left/image_raw
    frame_id: left_camera
  right:
    topic: /right/image_raw
    frame_id: right_camera
  baseline: 0.1
```

---

## 3.4 Running VSLAM

```bash
# Terminal 1: Isaac Sim
python ros2_bridge_setup.py

# Terminal 2: VSLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# Terminal 3: Teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 3.5 Visualization in RViz2

```bash
rviz2
# Add: TF, Odometry, Path, PointCloud2
```

---

## 3.6 Save/Load Maps

```bash
# Save
ros2 service call /visual_slam/save_map

# Load
ros2 service call /visual_slam/load_map
```

---

## Exercises

### Exercise 1: Run VSLAM
Launch and build a map for 5 minutes.

### Exercise 2: Save Map
Save and reload the map.

### Exercise 3: Benchmark
Compare GPU vs CPU performance.

---

## Summary

You learned:
- VSLAM fundamentals
- Isaac ROS configuration
- Map building and visualization
- Performance benchmarking

### Next Steps

**Chapter 4** covers depth perception and 3D point cloud mapping.

---

**Tags**: vslam, isaac-ros, gpu-acceleration, mapping
