---
title: "Ch2: Synthetic Data Generation"
description: "Master synthetic data generation in Isaac Sim with domain randomization, automated dataset creation, and dataset export for training perception models."
sidebar_position: 2
---

# Chapter 2: Synthetic Data Generation

## Introduction

Synthetic data generation is a cornerstone of modern robotics AI development. In this chapter, you'll learn how to leverage Isaac Sim's photorealistic rendering to create large-scale, perfectly labeled datasets.

### Learning Objectives

1. Configure the Isaac Sim Synthetic Data Generation (SDG) pipeline
2. Generate RGB, depth, semantic segmentation, and instance segmentation data
3. Apply domain randomization to increase dataset diversity
4. Automate batch generation of thousands of frames
5. Export datasets in standard formats (COCO, Pascal VOC)
6. Validate and visualize synthetic datasets

### Prerequisites

- Chapter 1 completed - Isaac Sim installation, USD models, basic simulation
- Python knowledge - File I/O, NumPy arrays, JSON handling
- Understanding of perception tasks - Object detection, segmentation

### Estimated Time

2-3 hours (including dataset generation and validation)

---

## 2.1 Synthetic Data Types

### RGB Images

Photorealistic color images rendered with ray-traced lighting and materials.

**Use cases:**
- Training object detection models (YOLO, Faster R-CNN)
- Visual servoing and tracking
- Scene understanding

**Format**: PNG or JPEG, 8-bit per channel (RGB)

### Depth Maps

Depth information for each pixel (distance from camera in meters).

**Use cases:**
- 3D reconstruction
- Obstacle detection for navigation
- Grasp pose estimation

**Format**: NumPy arrays (float32) or 16-bit PNG

### Semantic Segmentation

Per-pixel class labels (e.g., person, floor, wall, chair).

**Use cases:**
- Training segmentation models
- Scene parsing for mobile robots
- Human-robot interaction safety

**Format**: PNG with color mapping or NumPy arrays (int32)

---

## 2.2 Isaac Sim Replicator Framework

The Omniverse Replicator framework enables programmatic synthetic data generation.

```python
import omni.replicator.core as rep

# Create render product (camera view)
camera_path = "/World/Humanoid/head/left_camera"
rp = rep.create.render_product(camera_path, (1280, 720))

# Attach annotators
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach([rp])

depth = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
depth.attach([rp])

segmentation = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
segmentation.attach([rp])

# Get data
rgb_data = rgb.get_data()
depth_data = depth.get_data()
seg_data = segmentation.get_data()
```

---

## 2.3 Domain Randomization

Domain randomization increases dataset diversity, leading to better model generalization.

### Lighting Randomization

```python
import random

# Randomize light intensity
light_intensity = random.uniform(500, 3000)
rep.modify.attribute("/World/Light", "intensity", light_intensity)

# Randomize light color
light_color = (random.uniform(0.8, 1.0), 
               random.uniform(0.8, 1.0), 
               random.uniform(0.9, 1.0))
rep.modify.attribute("/World/Light", "color", light_color)
```

### Pose Randomization

```python
# Randomize robot position
x = random.uniform(-2.0, 2.0)
y = random.uniform(-2.0, 2.0)
z = 1.0  # Keep fixed height
rep.modify.pose("/World/Humanoid", position=(x, y, z))

# Randomize robot rotation
yaw = random.uniform(-180, 180)
rep.modify.pose("/World/Humanoid", rotation=(0, 0, yaw))
```

### Texture Randomization

```python
# Randomize ground texture color
ground_color = (random.uniform(0.2, 0.9), 
                random.uniform(0.2, 0.9), 
                random.uniform(0.2, 0.9))
rep.modify.attribute("/World/groundPlane", "primvars:displayColor", [ground_color])
```

---

## 2.4 Batch Data Generation

See `assets/code/isaac_sim/synthetic_data_pipeline.py` for complete implementation.

```python
for frame_id in range(num_frames):
    # Apply randomization
    randomize_scene()
    
    # Step simulation
    world.step(render=True)
    
    # Capture data
    rgb_data = rgb_annotator.get_data()
    depth_data = depth_annotator.get_data()
    seg_data = seg_annotator.get_data()
    
    # Save to disk
    save_frame(frame_id, rgb_data, depth_data, seg_data, output_dir)
    
    print(f"Frame {frame_id + 1}/{num_frames} captured")
```

---

## 2.5 Dataset Export Formats

### COCO Format

Common Objects in Context (COCO) format for object detection:

```json
{
  "images": [
    {
      "id": 1,
      "file_name": "rgb_000001.png",
      "width": 1280,
      "height": 720
    }
  ],
  "annotations": [
    {
      "id": 1,
      "image_id": 1,
      "category_id": 1,
      "bbox": [100, 150, 200, 300],
      "area": 60000,
      "iscrowd": 0
    }
  ],
  "categories": [
    {"id": 1, "name": "person"},
    {"id": 2, "name": "chair"}
  ]
}
```

### Pascal VOC Format

```xml
<annotation>
  <filename>rgb_000001.png</filename>
  <size>
    <width>1280</width>
    <height>720</height>
  </size>
  <object>
    <name>person</name>
    <bndbox>
      <xmin>100</xmin>
      <ymin>150</ymin>
      <xmax>300</xmax>
      <ymax>450</ymax>
    </bndbox>
  </object>
</annotation>
```

---

## Exercises

### Exercise 1: Generate 100-Frame Dataset (30 minutes)

Run `python synthetic_data_pipeline.py --num-frames 100 --output ./my_dataset`

**Success Criteria:**
- 100 RGB PNG files (approx 500KB each)
- 100 depth NPY files (approx 3.6MB each)
- Total size: approx 400MB

### Exercise 2: Domain Randomization (45 minutes)

Modify the pipeline to add lighting and pose randomization.

### Exercise 3: COCO Format Export (60 minutes)

Export synthetic dataset to COCO format for training object detectors.

---

## Summary

In this chapter, you learned:

- Configured the SDG pipeline with multiple annotators
- Applied domain randomization for lighting, poses, and textures
- Automated batch generation of labeled frames
- Exported datasets in COCO and Pascal VOC formats
- Validated dataset quality

### Key Takeaways

1. Synthetic data is perfectly labeled - no manual annotation required
2. Domain randomization improves model generalization
3. Automate everything for reproducible datasets
4. Export in standard formats for ML frameworks

### Next Steps

In **Chapter 3: VSLAM with Isaac ROS**, you will use depth and RGB data to implement Visual SLAM for real-time localization and mapping.

---

**Tags**: synthetic-data, domain-randomization, dataset-generation, coco-format, replicator
