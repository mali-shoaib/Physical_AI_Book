#!/usr/bin/env python3
"""
Enable ROS 2 bridge in Isaac Sim and configure topic publishing.

This script demonstrates how to programmatically enable the ROS 2 bridge
extension, configure camera topic publishing, and verify connectivity.

Usage:
    # Terminal 1 - Source ROS 2 and run this script:
    source /opt/ros/humble/setup.bash
    python ros2_bridge_setup.py
    
    # Terminal 2 - Verify topics:
    source /opt/ros/humble/setup.bash
    ros2 topic list
    ros2 topic hz /left/image_raw

Requirements:
    - ROS 2 Humble installed and sourced
    - Isaac Sim 2024.1+
"""

import sys
import os

# Verify ROS 2 environment
if "ROS_DISTRO" not in os.environ:
    print("ERROR: ROS 2 environment not sourced!")
    print("Please run: source /opt/ros/humble/setup.bash")
    sys.exit(1)

print(f"ROS 2 Distribution: {os.environ['ROS_DISTRO']}")
print(f"ROS Domain ID: {os.environ.get('ROS_DOMAIN_ID', '0')}")

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

# Import after SimulationApp launch
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.graph.core as og
import omni.ext
from pathlib import Path


def enable_ros2_bridge():
    """Enable the ROS 2 Bridge extension."""
    print("\nEnabling ROS 2 Bridge extension...")
    
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    extension_name = "omni.isaac.ros2_bridge"
    
    # Check if already enabled
    if extension_manager.is_extension_enabled(extension_name):
        print(f"✓ {extension_name} already enabled")
        return True
    
    # Enable extension
    try:
        extension_manager.set_extension_enabled(extension_name, True)
        print(f"✓ {extension_name} enabled successfully")
        return True
    except Exception as e:
        print(f"✗ Failed to enable {extension_name}: {e}")
        return False


def create_ros2_camera_graph(camera_path, topic_name, frame_id="camera"):
    """
    Create an action graph to publish camera data to ROS 2.
    
    Args:
        camera_path (str): USD path to camera (e.g., "/World/Humanoid/head/left_camera")
        topic_name (str): ROS 2 topic name (e.g., "/left/image_raw")
        frame_id (str): TF frame ID for camera
    """
    print(f"\nConfiguring ROS 2 camera publisher:")
    print(f"  Camera: {camera_path}")
    print(f"  Topic: {topic_name}")
    print(f"  Frame ID: {frame_id}")
    
    try:
        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {
                "graph_path": f"/ActionGraph_{topic_name.replace('/', '_')}",
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
                    ("ROS2CameraHelper.inputs:topicName", topic_name),
                    ("ROS2CameraHelper.inputs:frameId", frame_id),
                    ("ROS2CameraHelper.inputs:cameraPath", camera_path),
                    ("ROS2CameraHelper.inputs:type", "rgb"),
                ],
            },
        )
        print(f"✓ Action graph created for {topic_name}")
        return graph
    except Exception as e:
        print(f"✗ Failed to create action graph: {e}")
        return None


def create_clock_publisher():
    """Create ROS 2 /clock publisher for simulation time synchronization."""
    print("\nConfiguring ROS 2 /clock publisher...")
    
    try:
        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {
                "graph_path": "/ActionGraph_Clock",
                "evaluator_name": "execution",
            },
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ROS2PublishClock.inputs:execIn"),
                ],
            },
        )
        print("✓ /clock publisher created")
        return graph
    except Exception as e:
        print(f"✗ Failed to create clock publisher: {e}")
        return None


def main():
    """Main function."""
    # Enable ROS 2 bridge
    if not enable_ros2_bridge():
        print("\nFailed to enable ROS 2 bridge. Exiting.")
        simulation_app.close()
        return 1
    
    # Initialize World
    world = World()
    
    # Load humanoid with stereo cameras
    model_path = str(Path(__file__).parent.parent.parent / "models" / "humanoid_stereo_cameras.usda")
    print(f"\nLoading humanoid model: {model_path}")
    
    try:
        add_reference_to_stage(usd_path=model_path, prim_path="/World/Humanoid")
        print("✓ Humanoid model loaded")
    except Exception as e:
        print(f"✗ Failed to load model: {e}")
        simulation_app.close()
        return 1
    
    # Reset world
    world.reset()
    
    # Configure ROS 2 publishers for stereo cameras
    left_graph = create_ros2_camera_graph(
        camera_path="/World/Humanoid/head/left_camera",
        topic_name="/left/image_raw",
        frame_id="left_camera"
    )
    
    right_graph = create_ros2_camera_graph(
        camera_path="/World/Humanoid/head/right_camera",
        topic_name="/right/image_raw",
        frame_id="right_camera"
    )
    
    # Create clock publisher
    clock_graph = create_clock_publisher()
    
    if not (left_graph and right_graph and clock_graph):
        print("\n✗ Failed to configure all ROS 2 publishers")
        simulation_app.close()
        return 1
    
    print("\n" + "="*60)
    print("ROS 2 Bridge Configuration Complete!")
    print("="*60)
    print("\nVerify topics in another terminal:")
    print("  ros2 topic list")
    print("  ros2 topic hz /left/image_raw")
    print("  ros2 topic hz /right/image_raw")
    print("  ros2 topic hz /clock")
    print("\nVisualize in RViz2:")
    print("  rviz2")
    print("  Add -> Image -> Topic: /left/image_raw")
    print("\nPress Ctrl+C to exit.")
    
    # Run simulation
    try:
        while simulation_app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
    
    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
