---
title: "Ch5: Nav2 Integration"
description: "Integrate Isaac ROS perception with Nav2 path planning for autonomous bipedal navigation. Configure costmaps, planners, and recovery behaviors for humanoid robots."
sidebar_position: 5
---

# Chapter 5: Nav2 Integration

## Introduction

Nav2 (Navigation2) is the ROS 2 navigation stack. This chapter integrates Isaac ROS perception with Nav2 for autonomous humanoid navigation.

### Learning Objectives

1. Understand Nav2 architecture
2. Configure costmaps for humanoid robots
3. Integrate VSLAM and depth perception
4. Tune humanoid-specific parameters
5. Send navigation goals
6. Implement recovery behaviors
7. Achieve 85%+ navigation success rate

### Prerequisites

- Chapters 1-4 completed
- Understanding of path planning
- Familiarity with behavior trees

### Estimated Time

3-4 hours

---

## 5.1 Nav2 Architecture

Nav2 consists of:

1. **Global Planner** - Long-term path planning
2. **Local Controller** - Short-term trajectory following
3. **Costmaps** - Obstacle representation
4. **Behavior Trees** - High-level logic and recovery
5. **Lifecycle Manager** - Node state management

---

## 5.2 Costmap Configuration

Costmaps represent obstacles and free space.

### Global Costmap

```yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 1.0
  publish_frequency: 1.0
  resolution: 0.05
  
  plugins:
    - static_layer
    - obstacle_layer
    - inflation_layer
  
  static_layer:
    map_topic: /map
  
  obstacle_layer:
    observation_sources: vslam_cloud depth_cloud
    vslam_cloud:
      topic: /visual_slam/vis/observations_cloud
      sensor_frame: left_camera
    depth_cloud:
      topic: /depth/points
      sensor_frame: depth_camera
  
  inflation_layer:
    inflation_radius: 0.55
    cost_scaling_factor: 3.0
```

### Local Costmap

```yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  resolution: 0.02
  width: 3.0
  height: 3.0
  
  plugins:
    - obstacle_layer
    - inflation_layer
```

---

## 5.3 Humanoid-Specific Parameters

Humanoids require special tuning:

### Velocity Limits

```yaml
controller:
  max_vel_x: 0.5  # m/s (walking speed)
  max_vel_theta: 0.8  # rad/s (turning)
  min_vel_x: 0.1  # Minimum forward velocity
  
  # Acceleration limits
  acc_lim_x: 0.3
  acc_lim_theta: 1.0
```

### Footprint

```yaml
footprint: [
  [-0.2, -0.15],  # Back-left
  [-0.2, 0.15],   # Back-right
  [0.2, 0.15],    # Front-right
  [0.2, -0.15]    # Front-left
]
```

### TEB Controller

Time Elastic Band (TEB) works well for humanoids:

```yaml
controller_server:
  controller_plugins: ["FollowPath"]
  
  FollowPath:
    plugin: "teb_local_planner::TebLocalPlannerROS"
    
    # Humanoid-specific
    max_vel_x: 0.5
    max_vel_theta: 0.8
    footprint_model:
      type: "polygon"
      vertices: [[-0.2, -0.15], [-0.2, 0.15], [0.2, 0.15], [0.2, -0.15]]
    
    # Optimization
    no_inner_iterations: 5
    no_outer_iterations: 4
    optimization_activate: True
    
    # Obstacles
    min_obstacle_dist: 0.3
    include_costmap_obstacles: True
```

---

## 5.4 Launching Nav2

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }]
        ),
        
        # Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            parameters=['controller_params.yaml']
        ),
        
        # Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            parameters=['planner_params.yaml']
        ),
        
        # Behavior server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            parameters=['behavior_params.yaml']
        ),
        
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            parameters=['bt_navigator_params.yaml']
        ),
    ])
```

---

## 5.5 Sending Navigation Goals

### Via RViz2

1. Open RViz2
2. Click "2D Nav Goal" button
3. Click destination, drag to set orientation

### Programmatically

```python
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

navigator = BasicNavigator()

# Set goal
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.header.stamp = navigator.get_clock().now().to_msg()
goal_pose.pose.position.x = 2.0
goal_pose.pose.position.y = 1.0
goal_pose.pose.orientation.w = 1.0

# Navigate
navigator.goToPose(goal_pose)

# Wait for result
while not navigator.isTaskComplete():
    feedback = navigator.getFeedback()
    print(f"Distance remaining: {feedback.distance_remaining:.2f}m")

result = navigator.getResult()
if result == TaskResult.SUCCEEDED:
    print("Goal reached!")
```

---

## 5.6 Behavior Trees and Recovery

Default behavior tree structure:

```
NavigateToPose
├── ComputePathToPose
├── FollowPath
└── Recovery (if path following fails)
    ├── ClearCostmap
    ├── Spin
    ├── BackUp
    └── Wait
```

### Custom Recovery for Humanoids

```xml
<BehaviorTree>
  <NavigateToPose>
    <RateController hz="1.0">
      <RecoveryNode>
        <PipelineSequence>
          <ComputePathToPose/>
          <FollowPath/>
        </PipelineSequence>
        <ReactiveFallback>
          <GoalUpdated/>
          <ClearEntireCostmap/>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
          <ReplanPath/>
        </ReactiveFallback>
      </RecoveryNode>
    </RateController>
  </NavigateToPose>
</BehaviorTree>
```

---

## 5.7 Performance Monitoring

```python
def monitor_navigation_performance(navigator, num_trials=10):
    success_count = 0
    distances = []
    times = []
    
    for i in range(num_trials):
        goal = generate_random_goal()
        start_time = time.time()
        
        navigator.goToPose(goal)
        while not navigator.isTaskComplete():
            time.sleep(0.1)
        
        result = navigator.getResult()
        elapsed = time.time() - start_time
        
        if result == TaskResult.SUCCEEDED:
            success_count += 1
            times.append(elapsed)
    
    success_rate = success_count / num_trials
    print(f"Success rate: {success_rate * 100:.1f}%")
    print(f"Average time: {np.mean(times):.2f}s")
    
    return success_rate
```

---

## Exercises

### Exercise 1: Configure Nav2
Set up costmaps and launch Nav2.

### Exercise 2: Navigate to Goal
Send navigation goals via RViz2.

### Exercise 3: Measure Success Rate
Run 10 navigation trials, achieve 85%+ success.

---

## Summary

You learned:
- Nav2 architecture and components
- Costmap configuration for humanoids
- Integration with Isaac ROS perception
- Sending navigation goals
- Behavior tree recovery strategies

### Next Steps

**Chapter 6** brings everything together in an end-to-end capstone project.

---

**Tags**: nav2, path-planning, autonomous-navigation, costmaps, behavior-trees
