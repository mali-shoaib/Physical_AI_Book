---
title: "Ch4: Depth Perception & 3D Mapping"
description: "Process depth sensor data to create 3D point cloud maps. Learn stereo disparity, point cloud processing, and 3D navigation for complex environments."
sidebar_position: 4
---

# Chapter 4: Depth Perception & 3D Mapping

## Introduction

Depth perception is essential for navigating complex 3D environments. This chapter covers processing depth sensor data to create 3D point cloud maps for humanoid robots.

### Learning Objectives

1. Understand depth sensing technologies
2. Compute stereo disparity maps
3. Generate 3D point clouds
4. Filter and downsample point clouds
5. Visualize point clouds in RViz2
6. Navigate stairs and uneven terrain

### Prerequisites

- Chapters 1-3 completed
- Understanding of 3D geometry
- Familiarity with NumPy

### Estimated Time

2-3 hours

---

## 4.1 Depth Sensing Technologies

### Comparison

| Technology | Range | Accuracy | Outdoor | Cost |
|------------|-------|----------|---------|------|
| Stereo | 0.5-10m | Good | Yes | Low |
| ToF | 0.5-5m | Excellent | Limited | Medium |
| LiDAR | 0.1-100m | Excellent | Yes | High |

Stereo cameras provide good balance for humanoid robots.

---

## 4.2 Stereo Disparity

Disparity measures pixel offset between left and right images.

### Disparity to Depth Conversion

```
depth = (focal_length * baseline) / disparity
```

Where:
- focal_length: Camera focal length (pixels)
- baseline: Distance between cameras (meters)
- disparity: Pixel offset

---

## 4.3 Isaac ROS Stereo Image Proc

GPU-accelerated stereo processing:

```bash
sudo apt-get install ros-humble-isaac-ros-stereo-image-proc
```

Launch file:

```python
Node(
    package='isaac_ros_stereo_image_proc',
    executable='isaac_ros_stereo_image_proc',
    parameters=[{
        'approximate_sync': True,
        'use_system_default_qos': False,
    }]
)
```

---

## 4.4 Generating Point Clouds

Convert depth image to 3D points:

```python
import numpy as np

def depth_to_pointcloud(depth_image, intrinsics):
    h, w = depth_image.shape
    fx, fy = intrinsics['fx'], intrinsics['fy']
    cx, cy = intrinsics['cx'], intrinsics['cy']
    
    # Create pixel coordinates
    u = np.arange(w)
    v = np.arange(h)
    u, v = np.meshgrid(u, v)
    
    # Convert to 3D
    z = depth_image
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    
    points = np.stack([x, y, z], axis=-1)
    return points.reshape(-1, 3)
```

---

## 4.5 Point Cloud Filtering

### Voxel Grid Filter

Downsample for real-time processing:

```python
from scipy.spatial import cKDTree

def voxel_grid_filter(points, voxel_size=0.05):
    # Discretize to voxel grid
    voxels = np.floor(points / voxel_size).astype(int)
    
    # Keep one point per voxel
    _, unique_indices = np.unique(voxels, axis=0, return_index=True)
    return points[unique_indices]
```

### Statistical Outlier Removal

Remove noise:

```python
def statistical_outlier_removal(points, k=50, std_ratio=2.0):
    tree = cKDTree(points)
    distances, _ = tree.query(points, k=k)
    
    mean_dist = np.mean(distances, axis=1)
    threshold = np.mean(mean_dist) + std_ratio * np.std(mean_dist)
    
    mask = mean_dist < threshold
    return points[mask]
```

---

## 4.6 Visualization in RViz2

```bash
rviz2
# Add -> PointCloud2
# Topic: /depth/points
# Color: RGB8
```

---

## 4.7 3D Mapping for Stairs

Point clouds enable height-aware navigation:

1. Segment ground plane
2. Detect step edges
3. Plan footstep locations
4. Execute bipedal locomotion

---

## Exercises

### Exercise 1: Generate Point Cloud
Create point cloud from depth images.

### Exercise 2: Apply Filters
Downsample with voxel grid filter.

### Exercise 3: Visualize in RViz2
Display filtered point cloud.

---

## Summary

You learned:
- Depth sensing technologies
- Stereo disparity computation
- Point cloud generation and filtering
- 3D visualization

### Next Steps

**Chapter 5** integrates depth perception with Nav2 for autonomous navigation.

---

**Tags**: depth-perception, point-clouds, stereo-vision, 3d-mapping
