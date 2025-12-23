---
title: "Ch1: Isaac Sim Basics"
description: "Learn the fundamentals of NVIDIA Isaac Sim, including installation, UI navigation, robot model loading, and synthetic sensor data generation. Students will set up Isaac Sim, load humanoid robots, and run basic simulations."
sidebar_position: 1
---

# Chapter 1: Isaac Sim Basics

## Introduction

Welcome to Module 3 of Physical AI & Humanoid Robotics! In this module, we explore **NVIDIA Isaac Sim**, the photorealistic robotics simulation platform that enables you to train and test AI-powered robots in virtual environments before deploying them to the physical world.

Isaac Sim is built on NVIDIA Omniverse, leveraging ray-traced graphics and GPU-accelerated physics simulation to create digital twins of robots and environments with unparalleled fidelity. This chapter introduces the foundational concepts and hands-on skills you need to start building perception pipelines for humanoid robots.

### Learning Objectives

By the end of this chapter, you will be able to:

1. **Install and configure** NVIDIA Isaac Sim 2024.1+ on Ubuntu 22.04
2. **Navigate the Isaac Sim UI** including viewport, scene hierarchy, and property panels
3. **Load USD humanoid robot models** into Isaac Sim with correct physics properties
4. **Run basic simulations** with proper timestep configuration and real-time factors
5. **Set up the ROS 2 bridge** to publish sensor data from Isaac Sim to ROS 2 topics
6. **Capture synthetic sensor data** (RGB images, depth maps) for perception pipelines
7. **Troubleshoot common issues** such as GPU memory errors and physics instability

### Prerequisites

Before starting this chapter, you should have completed:

- **Module 1: ROS 2 Basics (Ch1-3)** - Understanding of ROS 2 nodes, topics, and URDF
- **Module 2: Digital Twin (Ch1-3)** - Familiarity with simulation environments and sensor models
- **Basic Python programming** - Functions, classes, and file I/O
- **Ubuntu 22.04 command-line familiarity** - Navigating directories, running scripts

### Estimated Time

**2-3 hours** (including installation, hands-on exercises, and troubleshooting)

### Hardware Requirements

- **GPU**: NVIDIA RTX 3060 or better (8GB+ VRAM)
- **RAM**: 16GB minimum, 32GB recommended
- **Storage**: 20GB free disk space for Isaac Sim installation
- **OS**: Ubuntu 22.04 LTS (officially supported)

---

## 1.1 Isaac Sim Installation

### 1.1.1 System Requirements

NVIDIA Isaac Sim has strict hardware and software requirements to ensure optimal performance:

**GPU Requirements:**
- NVIDIA GPU with **Compute Capability 7.0+** (Volta, Turing, Ampere, Ada, or Hopper architectures)
- **8GB+ VRAM** (16GB recommended for complex scenes)
- Examples of compatible GPUs:
  - RTX 3060 (12GB VRAM) - Entry-level for students
  - RTX 4070 (12GB VRAM) - Mid-range performance
  - RTX 4080 (16GB VRAM) - High performance for research
  - RTX 4090 (24GB VRAM) - Maximum performance

**Software Requirements:**
- **Ubuntu 22.04 LTS** (officially supported; Windows not recommended for ROS 2 integration)
- **NVIDIA Driver**: Version 525.xx or later
- **CUDA Toolkit**: 11.8 or 12.x (installed automatically with Isaac Sim)
- **cuDNN**: 8.x (bundled with Isaac Sim)

### 1.1.2 Installation Steps

**Step 1: Verify GPU and Driver**

Check your NVIDIA GPU model and driver version:

```bash
nvidia-smi
```

You should see output similar to:

```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.129.03   Driver Version: 535.129.03   CUDA Version: 12.2   |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
| 30%   45C    P8    25W / 320W |   1024MiB / 16384MiB |      2%      Default |
+-------------------------------+----------------------+----------------------+
```

Ensure the driver version is **525.xx or higher**.

**Step 2: Download Isaac Sim**

Isaac Sim 2024.1 can be installed via:
1. **Omniverse Launcher** (recommended for students)
2. **Standalone tarball** (for servers without GUI)

For this course, we use the **Omniverse Launcher** method:

```bash
# Download Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run launcher
./omniverse-launcher-linux.AppImage
```

**Step 3: Install Isaac Sim via Omniverse Launcher**

1. Sign in with your NVIDIA account (free for students)
2. Navigate to the **Exchange** tab
3. Search for "Isaac Sim"
4. Click **Install** for Isaac Sim 2024.1 (or latest stable version)
5. Wait for download and installation (~15GB download, ~30GB installed)

Default installation path: `/home/<username>/.local/share/ov/pkg/isaac-sim-2024.1.0`

**Step 4: Verify Installation**

Launch Isaac Sim to verify installation:

```bash
# Navigate to Isaac Sim directory
cd ~/.local/share/ov/pkg/isaac-sim-2024.1.0

# Launch Isaac Sim
./isaac-sim.sh
```

If Isaac Sim launches successfully and displays the viewport, installation is complete.

### 1.1.3 Version Pinning (Important!)

For reproducibility in educational content, we use **Isaac Sim 2024.1** throughout this module. Newer versions may introduce breaking changes to APIs or USD formats.

To check your installed version:

```bash
cat ~/.local/share/ov/pkg/isaac-sim-2024.1.0/VERSION
```

---

## 1.2 Isaac Sim UI Tour

### 1.2.1 Main Interface Components

The Isaac Sim interface consists of four primary panels:

1. **Viewport** (center) - Photorealistic 3D scene rendering
2. **Stage** panel (left) - Scene hierarchy of USD primitives
3. **Property** panel (right) - Object properties and settings
4. **Content Browser** (bottom) - Asset library and file explorer

### 1.2.2 Viewport Controls

Navigate the 3D viewport with these mouse controls:

- **Orbit**: Hold **Alt + Left Mouse Button** and drag
- **Pan**: Hold **Alt + Middle Mouse Button** and drag
- **Zoom**: Scroll **Mouse Wheel** or **Alt + Right Mouse Button** and drag
- **Focus Object**: Select object in Stage panel, press **F** key

### 1.2.3 Stage Panel

The **Stage** panel displays the hierarchical structure of your USD scene:

```
World
├── defaultLight
├── Environment
│   └── GroundPlane
└── Humanoid (to be added)
    ├── torso
    ├── head
    ├── left_arm
    ├── right_arm
    ├── left_leg
    └── right_leg
```

Right-click on any object to:
- Toggle visibility (eye icon)
- Delete object
- Inspect USD properties

### 1.2.4 Property Panel

The **Property** panel shows editable properties for the selected object:

- **Transform**: Position, rotation, scale (XYZ coordinates)
- **Physics**: Mass, friction, restitution, collision settings
- **Rendering**: Material properties, textures, colors
- **Custom Attributes**: User-defined metadata

---

## 1.3 Loading USD Robot Models

### 1.3.1 What is USD?

**Universal Scene Description (USD)** is a 3D file format developed by Pixar for representing complex 3D scenes. Isaac Sim uses USD instead of URDF because:

- **Composition**: Modular scene assembly with references and payloads
- **Layering**: Non-destructive editing with multiple layers
- **Performance**: Optimized for large-scale scenes with millions of primitives
- **Rendering**: Integrates materials, textures, and lighting

### 1.3.2 USD vs URDF Comparison

| Feature | URDF (ROS) | USD (Isaac Sim) |
|---------|-----------|-----------------|
| **Format** | XML-based | Binary or ASCII |
| **Physics** | Basic rigid body | Advanced GPU-accelerated physics |
| **Sensors** | Limited sensor models | Photorealistic RGB, depth, LiDAR |
| **Rendering** | Basic visualization | Ray-traced rendering with RTX |
| **Composition** | Single file (with meshes) | Modular references and layers |

### 1.3.3 Loading Pre-Built USD Humanoid

In this course, we provide three pre-built USD humanoid models:

1. **`humanoid_base.usda`** - Basic capsule-based humanoid (torso, head, arms, legs)
2. **`humanoid_stereo_cameras.usda`** - Base model + stereo camera pair on head
3. **`humanoid_full_sensors.usda`** - Base + stereo cameras + IMU + odometry + LiDAR

**Manual Loading (GUI Method):**

1. Click **File → Import** in Isaac Sim menu
2. Navigate to `docs/module-3-isaac/assets/models/humanoid_stereo_cameras.usda`
3. Click **Import**
4. The humanoid appears in the viewport and Stage panel

**Programmatic Loading (Recommended):**

See `assets/code/isaac_sim/load_humanoid_model.py` for Python script:

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage

# Initialize simulation world
world = World()

# Load humanoid USD model
humanoid_path = "/path/to/humanoid_stereo_cameras.usda"
add_reference_to_stage(usd_path=humanoid_path, prim_path="/World/Humanoid")

# Reset world to apply physics
world.reset()

simulation_app.close()
```

### 1.3.4 Verifying Physics Properties

After loading, verify the humanoid has correct physics properties:

1. Select **Humanoid** in Stage panel
2. Check **Property** panel for:
   - `PhysicsRigidBodyAPI` applied to all body parts
   - `PhysicsMassAPI` with realistic masses (e.g., torso=50kg, head=5kg)
   - `PhysicsCollisionAPI` enabled for collision detection

---

## 1.4 Running Basic Simulation

### 1.4.1 Simulation Controls

Isaac Sim simulation is controlled via the toolbar:

- **Play** ▶️ - Start physics simulation
- **Pause** ⏸️ - Pause simulation (state preserved)
- **Stop** ⏹️ - Stop and reset simulation to initial state

### 1.4.2 Timestep Configuration

The **timestep** (Δt) determines physics update frequency:

- **Default**: 1/60 seconds (60 Hz) for real-time simulation
- **Smaller timestep** (1/120s): More accurate physics, slower performance
- **Larger timestep** (1/30s): Faster performance, less accurate physics

Set timestep programmatically:

```python
from omni.isaac.core import World

world = World(physics_dt=1/60.0)  # 60 Hz physics updates
```

### 1.4.3 Real-Time Factor

The **real-time factor** (RTF) indicates simulation speed relative to real time:

- **RTF = 1.0**: Simulation runs at real-time speed (ideal)
- **RTF < 1.0**: Simulation slower than real-time (complex scenes)
- **RTF > 1.0**: Simulation faster than real-time (simple scenes, fast-forward)

Check RTF in the Isaac Sim status bar (bottom-right corner).

### 1.4.4 Running a 10-Second Simulation

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World

# Initialize world
world = World(physics_dt=1/60.0)
world.reset()

# Run simulation for 10 seconds
for i in range(600):  # 60 Hz * 10 seconds = 600 steps
    world.step(render=True)

simulation_app.close()
```

---

## 1.5 ROS 2 Bridge Setup

### 1.5.1 Why Use the ROS 2 Bridge?

The **ROS 2 Bridge** enables Isaac Sim to:
- Publish sensor data (RGB images, depth, IMU) to ROS 2 topics
- Subscribe to ROS 2 commands (velocity, joint positions)
- Synchronize simulation clock with ROS 2 `/clock` topic

This allows seamless integration with ROS 2 perception pipelines (Isaac ROS VSLAM, Nav2).

### 1.5.2 Enabling the ROS 2 Extension

**Method 1: GUI (Interactive)**

1. Open Isaac Sim
2. Navigate to **Window → Extensions**
3. Search for "ROS 2 Bridge"
4. Enable **omni.isaac.ros2_bridge** extension

**Method 2: Python Script (Programmatic)**

```python
import omni.ext

# Enable ROS 2 Bridge extension
extension_manager = omni.kit.app.get_app().get_extension_manager()
extension_manager.set_extension_enabled("omni.isaac.ros2_bridge", True)
```

### 1.5.3 Configuring Topic Publishing

Publish stereo camera images from `humanoid_stereo_cameras.usda`:

```python
import omni.graph.core as og

# Create ROS 2 Camera Helper node
keys = og.Controller.Keys
(graph, nodes, _, _) = og.Controller.edit(
    {
        "graph_path": "/ActionGraph",
        "evaluator_name": "execution",
    },
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ROS2CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "ROS2CameraHelper.inputs:execIn"),
        ],
        keys.SET_VALUES: [
            ("ROS2CameraHelper.inputs:topicName", "/left/image_raw"),
            ("ROS2CameraHelper.inputs:cameraPath", "/World/Humanoid/head/left_camera"),
        ],
    },
)
```

### 1.5.4 Clock Synchronization

Isaac Sim publishes a `/clock` topic to synchronize ROS 2 nodes with simulation time:

```bash
# Verify /clock topic is publishing
ros2 topic hz /clock

# Expected output:
# average rate: 60.0
#   min: 0.0165s max: 0.0168s std dev: 0.00001s window: 60
```

---

## 1.6 Capturing Synthetic Sensor Data

### 1.6.1 Synthetic Data Advantages

Synthetic data generated in Isaac Sim offers:

1. **Perfect ground truth** - Exact depth, segmentation, and 6-DOF poses
2. **Domain randomization** - Generate diverse lighting, textures, and backgrounds
3. **Scalability** - Generate thousands of labeled frames automatically
4. **Safety** - Test edge cases (robot falls, collisions) without hardware damage

### 1.6.2 RGB Image Capture

Capture RGB images from the left camera:

```python
import numpy as np
from PIL import Image
from omni.isaac.core.utils.render_product import create_render_product

# Create render product for left camera
render_product = create_render_product(
    "/World/Humanoid/head/left_camera",
    (1280, 720)  # Width, height
)

# Get RGB data (RGBA format)
rgb_data = render_product.get_rgba()

# Convert to NumPy array
rgb_array = np.frombuffer(rgb_data, dtype=np.uint8).reshape((720, 1280, 4))

# Save as PNG (discard alpha channel)
img = Image.fromarray(rgb_array[:, :, :3])
img.save("rgb_frame_000.png")
```

### 1.6.3 Depth Map Capture

Capture depth maps from the stereo camera baseline:

```python
# Get depth data (distance in meters)
depth_data = render_product.get_depth()

# Convert to NumPy array
depth_array = np.frombuffer(depth_data, dtype=np.float32).reshape((720, 1280))

# Save as NumPy file
np.save("depth_frame_000.npy", depth_array)

# Visualize depth map (convert to 8-bit grayscale)
depth_vis = ((depth_array / depth_array.max()) * 255).astype(np.uint8)
Image.fromarray(depth_vis).save("depth_frame_000_vis.png")
```

### 1.6.4 Automated Dataset Generation

Generate 100 frames with RGB + depth:

```python
# See assets/code/isaac_sim/synthetic_data_pipeline.py for full implementation

from omni.isaac.core import World
import numpy as np
from PIL import Image

world = World()
world.reset()

output_dir = "./synthetic_dataset"
num_frames = 100

for frame_id in range(num_frames):
    # Step simulation
    world.step(render=True)

    # Capture RGB
    rgb_data = render_product.get_rgba()
    rgb_array = np.frombuffer(rgb_data, dtype=np.uint8).reshape((720, 1280, 4))
    Image.fromarray(rgb_array[:, :, :3]).save(f"{output_dir}/rgb_{frame_id:04d}.png")

    # Capture depth
    depth_data = render_product.get_depth()
    depth_array = np.frombuffer(depth_data, dtype=np.float32).reshape((720, 1280))
    np.save(f"{output_dir}/depth_{frame_id:04d}.npy", depth_array)

    print(f"Frame {frame_id + 1}/{num_frames} captured")

print(f"Dataset generated: {num_frames} frames in {output_dir}")
```

---

## 1.7 Troubleshooting Common Issues

### 1.7.1 GPU Driver Errors

**Symptom**: Isaac Sim crashes on launch with "Failed to initialize GPU" error

**Solutions**:
1. Update NVIDIA driver to 525.xx or later:
   ```bash
   sudo apt update
   sudo apt install nvidia-driver-535
   sudo reboot
   ```
2. Verify CUDA compatibility:
   ```bash
   nvidia-smi | grep "CUDA Version"
   ```

### 1.7.2 CUDA Out of Memory

**Symptom**: "CUDA out of memory" error during simulation

**Causes**:
- Scene too complex (too many objects or high-res textures)
- Multiple Isaac Sim instances running
- Insufficient VRAM (\<8GB)

**Solutions**:
1. Reduce scene complexity:
   ```python
   # Lower camera resolution
   render_product = create_render_product("/camera", (640, 480))  # Instead of 1280x720
   ```
2. Close other GPU applications (browsers with hardware acceleration, other simulations)
3. Monitor VRAM usage:
   ```bash
   watch -n 1 nvidia-smi
   ```

### 1.7.3 Physics Instability

**Symptom**: Robot "explodes" or vibrates uncontrollably

**Causes**:
- Timestep too large (>1/30s)
- Overlapping collision meshes
- Mass ratios too extreme (e.g., 1000kg object on 0.1kg object)

**Solutions**:
1. Reduce timestep:
   ```python
   world = World(physics_dt=1/120.0)  # Increase to 120 Hz
   ```
2. Check for overlapping collisions in Stage panel
3. Adjust mass properties in Property panel

### 1.7.4 ROS 2 Topics Not Publishing

**Symptom**: `ros2 topic list` does not show Isaac Sim topics

**Causes**:
- ROS 2 bridge extension not enabled
- ROS 2 environment not sourced before launching Isaac Sim
- Domain ID mismatch between Isaac Sim and ROS 2

**Solutions**:
1. Source ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ./isaac-sim.sh
   ```
2. Check ROS 2 domain ID:
   ```bash
   echo $ROS_DOMAIN_ID  # Should match Isaac Sim domain ID (default: 0)
   ```
3. Verify bridge is enabled:
   ```python
   extension_manager.is_extension_enabled("omni.isaac.ros2_bridge")  # Should return True
   ```

---

## Exercises

### Exercise 1: Basic Simulation (15 minutes)

**Objective**: Launch Isaac Sim, load the `humanoid_base.usda` model, and run a 30-second simulation.

**Steps**:
1. Launch Isaac Sim via `./isaac-sim.sh`
2. Import `docs/module-3-isaac/assets/models/humanoid_base.usda`
3. Press **Play** ▶️ to start simulation
4. Observe the humanoid falling under gravity
5. Press **Stop** ⏹️ after 30 seconds

**Success Criteria**:
- Humanoid loads without errors
- Simulation runs at 60 FPS (check status bar)
- No CUDA out of memory errors

### Exercise 2: Synthetic Data Collection (30 minutes)

**Objective**: Generate a dataset of 50 RGB frames from the stereo camera.

**Steps**:
1. Load `humanoid_stereo_cameras.usda`
2. Run `python assets/code/isaac_sim/capture_rgb_depth.py --num-frames 50`
3. Verify 50 PNG images are saved in `./output/` directory

**Success Criteria**:
- 50 RGB images (1280x720 resolution)
- File sizes ~500KB each (lossless PNG compression)
- Images show humanoid from camera perspective

### Exercise 3: ROS 2 Integration (45 minutes)

**Objective**: Publish stereo camera images to ROS 2 topics and visualize in RViz2.

**Steps**:
1. Enable ROS 2 bridge in Isaac Sim
2. Configure `/left/image_raw` topic publishing
3. Run `rviz2` in a separate terminal
4. Add **Image** display for `/left/image_raw` topic
5. Verify real-time video stream in RViz2

**Success Criteria**:
- `/left/image_raw` topic publishing at 60 Hz
- RViz2 displays real-time camera feed
- No latency >100ms between Isaac Sim and RViz2

---

## Summary

In this chapter, you learned the fundamentals of **NVIDIA Isaac Sim**:

✅ **Installed Isaac Sim 2024.1** on Ubuntu 22.04 with GPU driver verification
✅ **Navigated the Isaac Sim UI** including viewport, Stage panel, and Property panel
✅ **Loaded USD humanoid robot models** with physics properties and sensors
✅ **Ran basic simulations** with proper timestep and real-time factor configuration
✅ **Set up the ROS 2 bridge** to publish sensor data to ROS 2 topics
✅ **Captured synthetic RGB and depth data** for perception pipelines
✅ **Troubleshot common issues** such as GPU memory errors and physics instability

### Key Takeaways

1. **USD is the native 3D format** for Isaac Sim, offering advanced composition and rendering capabilities over URDF.
2. **The ROS 2 bridge** enables seamless integration between Isaac Sim and ROS 2 perception stacks like Isaac ROS.
3. **Synthetic data generation** provides perfect ground truth for training perception models without real-world data collection.
4. **GPU resources are critical** - monitor VRAM usage and optimize scene complexity for your hardware.

### Next Steps

In **Chapter 2: Synthetic Data Generation**, you will learn advanced techniques for domain randomization, automated dataset generation, and exporting datasets for training neural networks.

In **Chapter 3: VSLAM with Isaac ROS**, you will integrate the Isaac ROS Visual SLAM pipeline with your synthetic stereo camera data to perform real-time localization and mapping.

---

## Additional Resources

- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [USD Tutorials by Pixar](https://graphics.pixar.com/usd/docs/index.html)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
- [Isaac Sim Python API Reference](https://docs.omniverse.nvidia.com/py/isaacsim/index.html)

---

**Tags**: isaac-sim, usd-models, ros2-bridge, synthetic-data, sensor-simulation
