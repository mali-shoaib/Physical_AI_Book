# Research Findings: Module 2 - Digital Twin (Gazebo & Unity)

**Feature**: 002-digital-twin
**Date**: 2025-12-18
**Purpose**: Resolve technical unknowns for Module 2 implementation planning

## Overview

This document consolidates research findings for creating Module 2 educational content covering Gazebo physics simulation, Unity rendering, and sensor emulation for digital twin development.

---

## Decision 1: Gazebo Version Selection

**Decision**: Use **Gazebo Fortress (Ignition Gazebo)** as primary platform, with Gazebo Classic (11) as secondary option

**Rationale**:
- Gazebo Fortress is the officially supported simulation platform for ROS 2 Humble
- Modern architecture with modular design, better performance, and improved physics engines
- Gazebo Classic (11) is in maintenance mode with no new features
- Forward compatibility: teaching Gazebo Fortress prepares students for current industry standards
- However, Gazebo Classic is more widely deployed in educational settings currently

**Alternatives Considered**:
1. Gazebo Classic only - rejected due to legacy status and limited future support
2. Gazebo Garden (newer) - rejected due to less ROS 2 Humble integration maturity
3. Other simulators (Webots, CoppeliaSim) - rejected to maintain ROS 2 ecosystem focus

**Implementation Approach**:
- Primary content uses Gazebo Fortress with SDF 1.8+ format
- Sidebar notes explain Gazebo Classic equivalents where significantly different
- Launch file examples work with both versions where possible

**Sources**:
- https://gazebosim.org/docs (official Gazebo documentation)
- https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo.html
- https://github.com/gazebosim/ros_gz (ROS 2 Humble integration packages)

---

## Decision 2: World File Format (SDF vs .world)

**Decision**: Use **SDF (.sdf) format** for all world and model files

**Rationale**:
- SDF is the modern, actively maintained format (current version: SDF 1.9)
- Provides XML schema validation and better error messages
- Supports advanced features: nested models, composition, parameterization
- Better forward compatibility and tool support
- .world files are legacy Gazebo Classic format

**Key SDF Features for Education**:
```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="educational_world">
    <physics name="default" type="ode">
      <!-- Physics parameters -->
    </physics>
    <scene><!-- Visual settings --></scene>
    <include><!-- Model references --></include>
    <plugin><!-- Gazebo plugins --></plugin>
  </world>
</sdf>
```

**Alternatives Considered**:
- URDF for worlds - rejected, URDF is for robot models not environments
- .world format - rejected due to legacy status

---

## Decision 3: Unity Version and ROS 2 Integration

**Decision**: Use **Unity 2022 LTS** with **ROS-TCP-Connector** for ROS 2 integration

**Rationale**:
- Unity 2022 LTS provides long-term support (3+ years) ensuring stability for educational content
- ROS-TCP-Connector is the standard Unity-ROS 2 bridge maintained by Unity Technologies
- Unity Robotics Hub provides comprehensive packages: ROS-TCP-Connector, URDF Importer, Perception
- Unity 2023 LTS is newer but has less community documentation and examples
- TCP-based connection is simpler than alternatives (websockets, native ROS nodes)

**Key Unity Packages for Module 2**:
- **ROS-TCP-Connector**: Bidirectional ROS 2 message passing
- **URDF Importer**: Import robot models directly from URDF/Xacro
- **Perception Package**: Synthetic data generation (depth, segmentation, bounding boxes)
- **Article Templates** (optional): Pre-built robot interaction scenes

**Integration Architecture**:
```
Unity (Rendering/Visualization) <--TCP--> ROS-TCP-Endpoint <--ROS 2 Topics--> Robot Stack
```

**Alternatives Considered**:
1. Unity 2023 LTS - rejected due to limited educational resources and newer API changes
2. ROS# (ROS Sharp) - rejected, less actively maintained than Unity's official packages
3. Native ROS 2 node in Unity - rejected due to complexity for educational content
4. Websocket bridges - rejected due to additional configuration complexity

**Sources**:
- https://github.com/Unity-Technologies/Unity-Robotics-Hub
- https://github.com/Unity-Technologies/ROS-TCP-Connector
- https://docs.unity3d.com/ (Unity 2022 LTS documentation)

---

## Decision 4: Unity Physics vs Gazebo Physics

**Decision**: **Gazebo for physics simulation, Unity for visualization only**

**Rationale**:
- Gazebo provides more accurate robotics-specific physics (contact models, joint dynamics)
- Unity physics engine (PhysX/Havok) optimized for games, not robotics precision
- Separation of concerns: physics fidelity vs rendering quality
- Students should understand the tool-appropriate use case

**When to Use Each**:

| Use Case | Platform | Rationale |
|----------|----------|-----------|
| Physics-based control testing | Gazebo | Accurate joint dynamics, contact forces |
| Sensor simulation (LiDAR, depth) | Gazebo or Unity | Both support, choice depends on workflow |
| Vision system testing | Unity Perception | Photorealistic rendering, ground truth labels |
| Human-robot interaction UI | Unity | Rich UI tools, interactive 3D environments |
| Multi-robot coordination | Gazebo | Better distributed simulation support |

**Educational Approach**:
- Chapter 1: Gazebo for physics fundamentals
- Chapter 2: Unity for visualization and human interaction
- Chapter 3: Sensors in both platforms (comparison of capabilities)

---

## Decision 5: Sensor Simulation Plugins and Noise Models

**Decision**: Use **standard Gazebo ROS plugins** with realistic noise parameters

**Gazebo Sensor Plugins**:

### LiDAR (gazebo_ros_ray_sensor)

```xml
<sensor name="lidar" type="ray">
  <plugin name="gazebo_ros_ray_sensor" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_link</frame_name>
  </plugin>

  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1.0</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.12</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
    <!-- Noise model (Gaussian) -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </ray>

  <update_rate>10.0</update_rate>
</sensor>
```

**Realistic Parameters** (based on Velodyne VLP-16):
- Range: 0.3m to 100m
- Angular resolution: 0.1° to 0.4°
- Update rate: 5-20 Hz
- Range accuracy: ±3cm
- Noise: Gaussian with σ = 0.03m

### Depth Camera (gazebo_ros_camera)

```xml
<sensor name="depth_camera" type="depth">
  <plugin name="gazebo_ros_camera" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/image_raw:=camera/depth/image_raw</remapping>
      <remapping>~/camera_info:=camera/depth/camera_info</remapping>
      <remapping>~/points:=camera/depth/points</remapping>
    </ros>
    <frame_name>camera_depth_frame</frame_name>
  </plugin>

  <camera name="depth_camera">
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
    <!-- Depth noise -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>

  <update_rate>30.0</update_rate>
</sensor>
```

**Realistic Parameters** (based on Intel RealSense D435):
- Resolution: 640x480 to 1280x720
- Frame rate: 6-90 FPS (typically 30 FPS)
- Depth range: 0.3m to 10m
- Depth accuracy: <2% at 2m distance
- FOV: 87° x 58° (horizontal x vertical)

### IMU (gazebo_ros_imu_sensor)

```xml
<sensor name="imu" type="imu">
  <plugin name="gazebo_ros_imu_sensor" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=imu/data</remapping>
    </ros>
    <frame_name>imu_link</frame_name>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
  </plugin>

  <imu>
    <!-- Angular velocity noise (gyroscope) -->
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.009</stddev>  <!-- rad/s -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0005</bias_stddev>
        </noise>
      </x>
      <!-- Y and Z similar -->
    </angular_velocity>

    <!-- Linear acceleration noise (accelerometer) -->
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>  <!-- m/s² -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </x>
      <!-- Y and Z similar -->
    </linear_acceleration>
  </imu>

  <update_rate>100.0</update_rate>
</sensor>
```

**Realistic Parameters** (based on Bosch BMI088):
- Gyroscope range: ±2000 deg/s
- Accelerometer range: ±24g
- Update rate: 100-400 Hz (typically 100 Hz)
- Gyroscope noise: 0.014 deg/s/√Hz
- Accelerometer noise: 175 μg/√Hz
- Bias drift: <0.5 deg/s (gyro), <40 mg (accel)

**Noise Model Types**:
1. **Gaussian noise**: Random measurement error (white noise)
2. **Bias drift**: Slowly changing offset over time
3. **Update rate limits**: Sensor frequency constraints

**Unity Perception Package** (for depth/segmentation):
- Supports semantic segmentation with ground truth labels
- Instance segmentation for individual object tracking
- 2D/3D bounding boxes
- Depth and surface normals
- Keypoint annotation for pose estimation

**ROS 2 Message Formats**:
- LiDAR: `sensor_msgs/msg/LaserScan` or `sensor_msgs/msg/PointCloud2`
- Depth Camera: `sensor_msgs/msg/Image` (depth values as 16-bit or float32)
- IMU: `sensor_msgs/msg/Imu` (orientation, angular_velocity, linear_acceleration)

**RViz2 Visualization Configurations**:
- LaserScan: Use LaserScan display plugin with decay time
- PointCloud2: PointCloud2 display with intensity/height coloring
- Image: Image display or CameraInfo for calibration visualization
- IMU: Axes display for orientation, Marker for acceleration vectors

---

## Decision 6: Code Organization for Educational Content

**Decision**: Use **hybrid approach** - inline code for <20 lines, external files for larger examples

**Rationale**:
- Short examples inline maximize learning clarity (students see code + explanation together)
- External files for complete runnable examples (world files, launch files, complex scripts)
- Reduces maintenance burden (update one file vs many embedded snippets)
- Aligns with existing Module 1 pattern

**Directory Structure for Module 2**:

```
docs/
└── module-2-digital-twin/
    ├── ch1-gazebo-physics.md
    ├── ch2-unity-rendering.md
    ├── ch3-sensor-simulation.md
    └── assets/
        └── code/
            ├── worlds/
            │   ├── empty_world.sdf
            │   ├── physics_demo.sdf
            │   └── sensor_playground.sdf
            ├── models/
            │   ├── simple_robot/
            │   │   ├── model.config
            │   │   └── model.sdf
            │   └── sensor_platform/
            │       ├── model.config
            │       └── model.sdf
            ├── launch/
            │   ├── gazebo_physics.launch.py
            │   ├── unity_visualization.launch.py
            │   └── sensor_demo.launch.py
            └── scripts/
                ├── spawn_robot.py
                └── test_sensors.py
```

**Code Inclusion Strategy**:
- **Inline**: SDF snippets <20 lines, Python code <15 lines, configuration examples
- **External with import**: Complete world files, launch files, full Python nodes
- **Link to repository**: Complex Unity scenes (too large for docs), complete robot packages

**Version Pinning Strategy**:
- **Exact versions** for core dependencies in documentation:
  - ROS 2 Humble (20240412 or later)
  - Gazebo Fortress (6.x.x) or Gazebo Classic (11.x.x)
  - Unity 2022.3 LTS
- **Version ranges** only for auxiliary Python packages:
  - numpy>=1.21,<2.0
  - matplotlib>=3.5

**Validation Integration**:
- All code examples must pass `validate_examples.py` (Python syntax)
- All SDF files must pass `validate_urdf.py` (XML structure, even though it's SDF)
- Launch files tested with dry-run mode before inclusion
- Unity scenes validated by opening in Unity Editor

---

## Decision 7: Troubleshooting Documentation Pattern

**Decision**: Include troubleshooting subsections in each chapter + dedicated appendix

**Structure**:

**In-Chapter Troubleshooting** (after each major section):
```markdown
### Common Issues

**Issue**: Gazebo world file fails to load

**Symptoms**: `[Err] Unable to find uri[model://ground_plane]`

**Solution**:
1. Check GAZEBO_MODEL_PATH: `echo $GAZEBO_MODEL_PATH`
2. Verify model exists: `ls /usr/share/gazebo-11/models`
3. Source workspace: `source /opt/ros/humble/setup.bash`

**Prevention**: Always source ROS 2 and workspace setup before launching Gazebo
```

**Dedicated Troubleshooting Appendix** (module-level):
- Environment setup issues (PATH, ROS 2 sourcing)
- Plugin loading failures (library path, naming)
- Performance problems (framerate, physics instability)
- Connection issues (Unity-ROS 2 TCP, topic mismatches)

**Troubleshooting Checklist Pattern**:
1. Verify installation (version check commands)
2. Check environment variables
3. Test minimal example
4. Enable verbose logging
5. Consult official documentation

---

## Decision 8: Physics Parameters Teaching Progression

**Decision**: Teach physics in order of **observable impact** (basic → advanced)

**Chapter 1 Progression**:

**Section 1.1: Gravity and Basic Forces**
- Start with gravity direction/magnitude (immediately visible)
- Drop test: spawn box above ground, observe fall
- Teach: gravity vector, acceleration due to gravity

**Section 1.2: Contact Mechanics**
- Friction coefficients (affects sliding, robot mobility)
- Bouncing (restitution coefficient)
- Teach: surface properties determine interaction

**Section 1.3: Simulation Accuracy**
- Timestep (max_step_size)
- Solver iterations
- Teach: accuracy vs performance tradeoffs

**Section 1.4: Joint Dynamics**
- Joint limits, damping, friction
- Effort/velocity control
- Teach: actuator modeling for realistic robot behavior

**Section 1.5: Performance Optimization**
- Real-time factor tuning
- Mesh simplification
- Physics engine selection
- Teach: when to sacrifice accuracy for speed

---

## Summary of Key Technical Decisions

| Decision Area | Choice | Primary Rationale |
|---------------|--------|-------------------|
| Gazebo Version | Gazebo Fortress (primary) | Modern architecture, ROS 2 Humble native support |
| World Format | SDF 1.8+ (.sdf files) | Active development, better validation, forward compatibility |
| Unity Version | Unity 2022 LTS | Long-term support, mature ROS 2 integration |
| ROS 2 Bridge | ROS-TCP-Connector | Official Unity support, TCP simplicity |
| Physics Platform | Gazebo (not Unity) | Robotics-specific accuracy, better contact models |
| Sensor Plugins | Gazebo ROS packages | Standard, well-documented, realistic noise models |
| Code Organization | Hybrid (inline + external) | Balance learning clarity with maintainability |
| Version Pinning | Exact for core, ranges for auxiliary | Reproducibility while allowing minor updates |

---

## Open Questions / Future Research

1. **Unity ML-Agents Integration**: Not critical for Module 2, defer to Module 4 (VLA pipeline)
2. **Gazebo Harmonic/Garden**: Monitor for future module updates when ROS 2 Iron/Jazzy adoption increases
3. **Cloud Simulation**: NVIDIA Isaac Sim focus in Module 3, not needed for Module 2
4. **Performance Benchmarks**: Establish baseline performance metrics for example worlds (FPS, real-time factor)

---

## References

### Gazebo
- https://gazebosim.org/docs (Gazebo Fortress documentation)
- https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo.html
- https://github.com/gazebosim/ros_gz (ROS 2 Gazebo integration)
- https://classic.gazebosim.org/ (Gazebo Classic 11 documentation)

### Unity
- https://github.com/Unity-Technologies/Unity-Robotics-Hub
- https://github.com/Unity-Technologies/ROS-TCP-Connector
- https://github.com/Unity-Technologies/URDF-Importer
- https://github.com/Unity-Technologies/com.unity.perception
- https://docs.unity3d.com/ (Unity 2022 LTS documentation)

### ROS 2 Sensor Messages
- https://docs.ros.org/en/humble/p/sensor_msgs/

### Sensor Datasheets
- Velodyne VLP-16 (LiDAR reference)
- Intel RealSense D435 (Depth camera reference)
- Bosch BMI088 (IMU reference)

---

**Research Status**: Complete
**Next Phase**: Create data-model.md, contracts/, and quickstart.md (Phase 1)
