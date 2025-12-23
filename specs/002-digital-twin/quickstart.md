# Quickstart Guide: Module 2 - Digital Twin (Gazebo & Unity)

**Audience**: Content authors, technical writers, educators developing Module 2
**Purpose**: Step-by-step guide for creating and validating Module 2 educational content
**Last Updated**: 2025-12-18

---

## Overview

This guide provides workflows for:
1. Creating new chapters with proper frontmatter and structure
2. Adding Gazebo world files and simulation examples
3. Setting up Unity scenes for robot visualization
4. Configuring virtual sensors with realistic noise models
5. Validating all content before publishing

---

## Prerequisites

Before creating Module 2 content, ensure you have:

**Required Software**:
- ROS 2 Humble (20240412 or later)
- Gazebo Fortress (6.x.x) or Gazebo Classic (11.x.x)
- Unity 2022.3 LTS
- Python 3.10+
- Node.js 18+ (for Docusaurus build)

**Repository Setup**:
```bash
# Clone and setup repository
git clone <repository-url>
cd books
npm install

# Install validation dependencies
python -m pip install --upgrade pip
# (No additional Python packages required for basic validation)

# Source ROS 2
source /opt/ros/humble/setup.bash
```

**Knowledge Requirements**:
- Completed Module 1 (understand ROS 2 basics, URDF, launch files)
- Familiarity with Gazebo simulation (world files, plugins)
- Basic Unity Editor usage (scenes, GameObjects, components)

---

## Workflow 1: Create a New Chapter

### Step 1: Create Markdown File

```bash
# Chapter naming convention: ch{N}-{topic-slug}.md
cd docs/module-2-digital-twin
touch ch4-advanced-sensors.md
```

### Step 2: Add Frontmatter

Copy from `specs/002-digital-twin/contracts/chapter-frontmatter-schema.yaml` and customize:

```yaml
---
id: ch4-advanced-sensors
title: "Chapter 4: Advanced Sensor Configurations"
sidebar_label: "Ch4: Advanced Sensors"
sidebar_position: 4
description: "Configure complex sensor arrays and multi-modal perception systems"
keywords:
  - sensors
  - lidar
  - depth camera
  - sensor fusion
learning_objectives:
  - "Configure multi-sensor arrays on humanoid robots"
  - "Implement sensor fusion for improved perception accuracy"
  - "Optimize sensor update rates for real-time performance"
prerequisites:
  - "Chapter 3: Sensor Simulation (basic sensor configuration)"
  - "Understanding of coordinate frame transformations"
estimated_time: 75
difficulty_level: advanced
---
```

### Step 3: Structure Content

Use this standard chapter structure:

```markdown
# {Chapter Title}

:::info Learning Path
This chapter builds on [previous chapter] and prepares you for [next topic].
:::

## Introduction

[1-2 paragraphs explaining chapter goals and real-world relevance]

## Section 1: {Topic Name}

### Theory

[Explain concepts with diagrams]

### Code Example

[Include runnable code with explanation]

### Practice Exercise

[Hands-on task for students]

## Section 2: {Next Topic}

[Repeat structure]

## Troubleshooting

[Common issues and solutions]

## Summary

[Key takeaways, 3-5 bullet points]

## Next Steps

[Link to next chapter]
```

### Step 4: Update Sidebar

Edit `sidebars.js` to add new chapter:

```javascript
{
  type: 'category',
  label: 'Module 2: Digital Twin',
  collapsed: false,
  items: [
    'module-2-digital-twin/ch1-gazebo-physics',
    'module-2-digital-twin/ch2-unity-rendering',
    'module-2-digital-twin/ch3-sensor-simulation',
    'module-2-digital-twin/ch4-advanced-sensors',  // NEW CHAPTER
  ],
},
```

---

## Workflow 2: Add Gazebo World Files

### Step 1: Create World File

```bash
# Create directory for code assets
mkdir -p docs/module-2-digital-twin/assets/code/worlds

# Copy template
cp specs/002-digital-twin/contracts/gazebo-world-template.sdf \
   docs/module-2-digital-twin/assets/code/worlds/my_new_world.sdf
```

### Step 2: Customize World

Edit `my_new_world.sdf`:
- Update `<world name="">` to match filename (without .sdf)
- Modify physics parameters for teaching concept
- Add models or include from model database
- Add educational comments explaining each section

**Example - Friction Testing World**:

```xml
<world name="friction_test">
  <physics name="default_physics" type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000.0</real_time_update_rate>
    <gravity>0 0 -9.81</gravity>

    <ode>
      <solver>
        <type>quick</type>
        <iters>50</iters>
      </solver>
    </ode>
  </physics>

  <!-- Ground plane with LOW friction (ice-like) -->
  <model name="icy_ground">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>100 100</size>
          </plane>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.1</mu>   <!-- Low friction -->
              <mu2>0.1</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <!-- ... visual element ... -->
    </link>
  </model>

  <!-- Box to slide on icy ground -->
  <model name="sliding_box">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="box_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.1</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.1</iyy><iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      <!-- ... collision and visual elements ... -->
    </link>
  </model>
</world>
```

### Step 3: Validate World File

```bash
# SDF schema validation (Gazebo Fortress)
gz sdf --check docs/module-2-digital-twin/assets/code/worlds/my_new_world.sdf

# Test loading in Gazebo
gz sim docs/module-2-digital-twin/assets/code/worlds/my_new_world.sdf

# For Gazebo Classic:
# gazebo docs/module-2-digital-twin/assets/code/worlds/my_new_world.world
```

### Step 4: Reference in Chapter

In your chapter markdown:

```markdown
## Friction Coefficient Experiments

This simulation demonstrates how friction affects robot mobility.

```sdf
<!-- Inline snippet showing key friction config -->
<surface>
  <friction>
    <ode>
      <mu>0.1</mu>   <!-- Low friction (ice) -->
      <mu2>0.1</mu2>
    </ode>
  </friction>
</surface>
```

**Complete World File**: [friction_test.sdf](assets/code/worlds/friction_test.sdf)

**Run the simulation**:
```bash
gz sim friction_test.sdf
```

Expected behavior: Box slides easily across ground with minimal resistance.
```

---

## Workflow 3: Add Unity Scenes

### Step 1: Create Unity Project

```bash
# Create Unity project (Unity 2022.3 LTS)
# Open Unity Hub → New Project → 3D (Built-in Render Pipeline)
# Name: PhysicalAI_Module2_Unity
# Location: docs/module-2-digital-twin/assets/unity/
```

### Step 2: Install Unity Packages

In Unity Package Manager:
1. Add packages from Git URL:
   - `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
   - `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`
   - `https://github.com/Unity-Technologies/com.unity.perception.git`

2. Or use Package Manager UI:
   - Window → Package Manager
   - Click "+" → Add package from git URL
   - Paste URL and click "Add"

### Step 3: Configure ROS 2 Connection

Create `Assets/Scripts/ROSConnection.cs`:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class ROSConnection : MonoBehaviour
{
    void Start()
    {
        // Connect to ROS 2 endpoint
        ROSConnection.GetOrCreateInstance().Connect(
            "127.0.0.1",  // ROS TCP Endpoint IP
            10000         // Port
        );

        Debug.Log("Connected to ROS 2 via TCP");
    }
}
```

### Step 4: Create Scene

1. Create new scene: File → New Scene
2. Add environment models (floor, walls, furniture)
3. Import robot URDF: URDF → Import Robot from File
4. Add ROSConnection script to GameObject
5. Configure lighting (Directional Light + HDRI skybox)
6. Save scene: `Assets/Scenes/OfficeEnvironment.unity`

### Step 5: Document Scene in Chapter

```markdown
## Unity Scene Setup

**Scene**: OfficeEnvironment.unity

This scene demonstrates a humanoid robot in a typical office setting.

**Prerequisites**:
- Unity 2022.3 LTS installed
- ROS-TCP-Connector package
- ROS 2 Humble with `ros_tcp_endpoint` running

**Setup Instructions**:
1. Open Unity project: `docs/module-2-digital-twin/assets/unity/PhysicalAI_Module2_Unity`
2. Load scene: `Assets/Scenes/OfficeEnvironment.unity`
3. Start ROS 2 endpoint:
   ```bash
   ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
   ```
4. Press Play in Unity Editor

**Expected Behavior**: Robot model updates joint positions from ROS 2 `/joint_states` topic.
```

---

## Workflow 4: Configure Virtual Sensors

### Step 1: Use Sensor Schema

Reference `specs/002-digital-twin/contracts/sensor-config-schema.yaml` for sensor configuration structure.

### Step 2: Add LiDAR to Gazebo Model

Edit robot model SDF to add LiDAR sensor:

```xml
<link name="lidar_link">
  <pose relative_to="base_link">0 0 0.3 0 0 0</pose>

  <sensor name="front_lidar" type="ray">
    <plugin filename="libgazebo_ros_ray_sensor.so" name="gazebo_ros_ray_sensor">
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

      <!-- Realistic noise (3cm std dev) -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.03</stddev>
      </noise>
    </ray>

    <update_rate>10.0</update_rate>
    <visualize>true</visualize>
  </sensor>
</link>
```

### Step 3: Test Sensor Data

```bash
# Launch Gazebo with robot model
gz sim robot_with_lidar.sdf

# In another terminal, check ROS 2 topic
ros2 topic echo /robot/scan

# Visualize in RViz2
rviz2
# Add → LaserScan → Topic: /robot/scan
```

### Step 4: Document in Chapter

```markdown
## LiDAR Sensor Configuration

This section demonstrates adding a 2D LiDAR sensor to a robot model.

**Sensor Specifications** (based on Velodyne VLP-16):
- Range: 0.12m to 10m
- Angular resolution: 1° (360 samples)
- Update rate: 10 Hz
- Noise model: Gaussian with σ = 3cm

```xml
<!-- Key sensor configuration snippet -->
<sensor name="front_lidar" type="ray">
  <!-- ... -->
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.03</stddev>
  </noise>
</sensor>
```

**Validation**:
1. Launch simulation: `gz sim robot_with_lidar.sdf`
2. Verify topic publishes: `ros2 topic hz /robot/scan`
   - Expected: ~10 Hz
3. Check data format: `ros2 topic echo /robot/scan --once`
   - Expected: sensor_msgs/LaserScan with 360 ranges

**Troubleshooting**: If no data appears, check:
- Plugin loaded: Look for "libgazebo_ros_ray_sensor.so loaded" in Gazebo output
- Frame exists: `ros2 run tf2_tools view_frames.py` should show `lidar_link`
- Topic remapped correctly: Use `ros2 topic list` to verify `/robot/scan` exists
```

---

## Workflow 5: Validate Content

### Step 1: Run Validation Scripts

```bash
# From repository root

# Validate Python code examples
python scripts/validate_examples.py docs/module-2-digital-twin/

# Validate SDF/XML files
python scripts/validate_urdf.py docs/module-2-digital-twin/

# Validate imports
python scripts/validate_imports.py docs/module-2-digital-twin/

# Run all validations
python scripts/validate_all.py --skip-tests
```

### Step 2: Test Docusaurus Build

```bash
# Clear cache
npm run clear

# Build documentation
npm run build

# If build succeeds, test locally
npm run serve
# Open http://localhost:3000/books/
```

### Step 3: Manual Testing

**For Gazebo Examples**:
1. Launch each world file: `gz sim <world_file.sdf>`
2. Verify no errors in console output
3. Check physics behavior matches description
4. Test with verbose logging: `gz sim <world_file.sdf> -v 4`

**For Unity Scenes**:
1. Open scene in Unity Editor
2. Press Play, verify no errors in Console
3. Test ROS 2 connection (if applicable)
4. Check frame rate (should be >= 30 FPS)

**For Code Examples**:
1. Copy code to test environment
2. Execute and verify expected output
3. Test error conditions if documented

---

## Content Quality Checklist

Before committing new Module 2 content, verify:

**General**:
- [ ] Frontmatter complete with all required fields
- [ ] Learning objectives are specific and measurable (3-6 items)
- [ ] Prerequisites reference earlier content accurately
- [ ] Estimated time is realistic (60-120 min per chapter)

**Code Examples**:
- [ ] All Python code passes `validate_examples.py`
- [ ] All SDF files pass `validate_urdf.py` or `gz sdf --check`
- [ ] External files exist in `assets/code/` directory
- [ ] Inline examples are <20 lines for readability
- [ ] All examples have explanatory comments

**Simulations**:
- [ ] Gazebo worlds load without errors
- [ ] Physics parameters are realistic and explained
- [ ] Sensor configurations match real hardware specifications
- [ ] Noise models are documented with sources

**Documentation**:
- [ ] Sections follow standard structure (Theory → Example → Practice)
- [ ] Troubleshooting section included for complex topics
- [ ] All images have alt text for accessibility
- [ ] Links to official documentation (Gazebo, Unity, ROS 2) are current

**Testing**:
- [ ] Docusaurus build succeeds (`npm run build`)
- [ ] All code examples tested in clean environment
- [ ] Screenshots/GIFs are clear and annotated

---

## Common Troubleshooting

### Issue: Gazebo World Won't Load

**Symptom**: `[Err] Unable to find uri[model://...]`

**Solutions**:
1. Check GAZEBO_MODEL_PATH: `echo $GAZEBO_MODEL_PATH`
2. Add model path: `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/models`
3. Use inline model definitions instead of `<include>` for teaching examples

### Issue: Unity ROS Connection Fails

**Symptom**: "Could not connect to ROS TCP Endpoint"

**Solutions**:
1. Verify ROS TCP Endpoint is running:
   ```bash
   ros2 run ros_tcp_endpoint default_server_endpoint
   ```
2. Check IP/port in Unity: ROSConnectionPrefab → Inspector → ROS IP Address
3. Test connectivity: `telnet 127.0.0.1 10000`

### Issue: Validation Script Fails

**Symptom**: "No Python code blocks found" or "Invalid XML"

**Solutions**:
1. Check code block language identifier: \`\`\`python (not \`\`\`py or \`\`\`)
2. Validate XML manually: `xmllint --noout <file.sdf>`
3. Ensure files are in `docs/module-2-digital-twin/` subdirectory

---

## Additional Resources

- **Gazebo Documentation**: https://gazebosim.org/docs
- **Unity Robotics Hub**: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- **ROS 2 Humble Docs**: https://docs.ros.org/en/humble/
- **Module 1 Example Structure**: `docs/module-1-ros2/` (reference for consistency)
- **Spec and Research**: `specs/002-digital-twin/` (technical decisions and rationale)

---

**Questions or Issues?**
- Review planning documents: `specs/002-digital-twin/plan.md`
- Check data model: `specs/002-digital-twin/data-model.md`
- Consult research findings: `specs/002-digital-twin/research.md`

---

**Document Version**: 1.0
**Last Validated**: 2025-12-18
**Next Review**: After Phase 5 implementation (first chapter complete)
