#!/usr/bin/env python3
"""
Launch Isaac Sim programmatically with custom configuration.

This script demonstrates how to launch Isaac Sim from Python code,
configure simulation parameters, and verify the launch was successful.

Usage:
    python launch_isaac_sim.py [--headless] [--physics-dt 0.01667]

Requirements:
    - NVIDIA Isaac Sim 2024.1+ installed
    - NVIDIA GPU with 8GB+ VRAM
    - Ubuntu 22.04 LTS
"""

import argparse
import sys
from pathlib import Path

try:
    from omni.isaac.kit import SimulationApp
except ImportError:
    print("ERROR: Isaac Sim Python API not found!")
    print("Please ensure Isaac Sim is installed and PYTHONPATH is set correctly.")
    print("\nTo set PYTHONPATH:")
    print("  export PYTHONPATH=$PYTHONPATH:~/.local/share/ov/pkg/isaac-sim-2024.1.0")
    sys.exit(1)


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Launch Isaac Sim programmatically"
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run Isaac Sim in headless mode (no GUI)"
    )
    parser.add_argument(
        "--physics-dt",
        type=float,
        default=1/60.0,
        help="Physics timestep in seconds (default: 1/60 = 0.01667s)"
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1920,
        help="Window width in pixels (default: 1920)"
    )
    parser.add_argument(
        "--height",
        type=int,
        default=1080,
        help="Window height in pixels (default: 1080)"
    )
    return parser.parse_args()


def launch_isaac_sim(headless=False, physics_dt=1/60.0, width=1920, height=1080):
    """
    Launch Isaac Sim with specified configuration.
    
    Args:
        headless (bool): Run without GUI if True
        physics_dt (float): Physics simulation timestep in seconds
        width (int): Window width in pixels
        height (int): Window height in pixels
    
    Returns:
        SimulationApp: The launched simulation application instance
    """
    print(f"Launching Isaac Sim...")
    print(f"  Headless mode: {headless}")
    print(f"  Physics dt: {physics_dt:.5f}s ({1/physics_dt:.1f} Hz)")
    print(f"  Window size: {width}x{height}")
    
    # Configure simulation app
    config = {
        "headless": headless,
        "width": width,
        "height": height,
    }
    
    # Launch simulation app
    simulation_app = SimulationApp(config)
    
    # Import Isaac Sim modules after app is launched
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import create_new_stage
    
    # Create a new empty stage
    create_new_stage()
    
    # Initialize World with physics configuration
    world = World(physics_dt=physics_dt, rendering_dt=physics_dt)
    
    print("Isaac Sim launched successfully!")
    print(f"World initialized with physics_dt={world.get_physics_dt():.5f}s")
    
    return simulation_app, world


def main():
    """Main function."""
    args = parse_args()
    
    try:
        simulation_app, world = launch_isaac_sim(
            headless=args.headless,
            physics_dt=args.physics_dt,
            width=args.width,
            height=args.height
        )
        
        print("\nIsaac Sim is running. Press Ctrl+C to exit.")
        print("You can now interact with the simulation programmatically or via GUI.")
        
        # Keep the simulation running
        if not args.headless:
            print("\nClose the Isaac Sim window to exit.")
            while simulation_app.is_running():
                world.step(render=True)
        else:
            print("\nRunning headless simulation for 10 seconds...")
            for i in range(int(10 / args.physics_dt)):
                world.step(render=False)
                if i % 60 == 0:
                    print(f"  Simulation time: {i * args.physics_dt:.2f}s")
        
    except KeyboardInterrupt:
        print("\nShutting down Isaac Sim...")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        if 'simulation_app' in locals():
            simulation_app.close()
    
    print("Isaac Sim closed successfully.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
