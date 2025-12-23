#!/usr/bin/env python3
"""
Load USD humanoid robot model into Isaac Sim and verify physics properties.

This script demonstrates how to load a USD robot model, verify its physics
configuration, and inspect sensor attachments.

Usage:
    python load_humanoid_model.py --model humanoid_stereo_cameras.usda

Requirements:
    - Isaac Sim 2024.1+
    - USD humanoid models in assets/models/
"""

import argparse
import sys
from pathlib import Path

from omni.isaac.kit import SimulationApp

# Parse arguments before launching SimulationApp
parser = argparse.ArgumentParser(description="Load USD humanoid model")
parser.add_argument(
    "--model",
    type=str,
    default="humanoid_stereo_cameras.usda",
    choices=["humanoid_base.usda", "humanoid_stereo_cameras.usda", "humanoid_full_sensors.usda"],
    help="Which humanoid model to load"
)
parser.add_argument("--headless", action="store_true", help="Run without GUI")
args = parser.parse_args()

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": args.headless})

# Import Isaac modules after app launch
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import Usd, UsdGeom, UsdPhysics


def find_model_path(model_name):
    """Find the USD model file path."""
    # Try relative path from current directory
    relative_path = Path(__file__).parent.parent.parent / "models" / model_name
    if relative_path.exists():
        return str(relative_path.resolve())
    
    # Try absolute path
    absolute_path = Path("/docs/module-3-isaac/assets/models") / model_name
    if absolute_path.exists():
        return str(absolute_path)
    
    print(f"ERROR: Could not find {model_name}")
    print(f"  Tried: {relative_path}")
    print(f"  Tried: {absolute_path}")
    return None


def verify_physics_properties(prim_path):
    """Verify that the humanoid has correct physics properties."""
    stage = get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    
    if not prim:
        print(f"ERROR: Prim not found at {prim_path}")
        return False
    
    print(f"\n{'='*60}")
    print(f"Verifying physics properties for: {prim_path}")
    print(f"{'='*60}")
    
    # Check for rigid body API
    if UsdPhysics.RigidBodyAPI(prim):
        print("✓ PhysicsRigidBodyAPI applied")
    else:
        print("✗ PhysicsRigidBodyAPI NOT applied")
    
    # Check for mass API
    mass_api = UsdPhysics.MassAPI(prim)
    if mass_api:
        mass = mass_api.GetMassAttr().Get()
        print(f"✓ PhysicsMassAPI applied (mass: {mass} kg)")
    else:
        print("✗ PhysicsMassAPI NOT applied")
    
    # Check for collision API
    if UsdPhysics.CollisionAPI(prim):
        print("✓ PhysicsCollisionAPI applied")
    else:
        print("✗ PhysicsCollisionAPI NOT applied")
    
    return True


def list_sensors(prim_path):
    """List all sensors attached to the humanoid."""
    stage = get_current_stage()
    root_prim = stage.GetPrimAtPath(prim_path)
    
    print(f"\n{'='*60}")
    print(f"Sensors attached to {prim_path}")
    print(f"{'='*60}")
    
    sensors = []
    for prim in Usd.PrimRange(root_prim):
        prim_type = prim.GetTypeName()
        if prim_type in ["Camera", "Lidar"]:
            sensors.append((prim.GetPath(), prim_type))
            print(f"  {prim_type}: {prim.GetPath()}")
    
    if not sensors:
        print("  (No sensors found)")
    
    return sensors


def main():
    """Main function."""
    print(f"Loading model: {args.model}")
    
    # Find model path
    model_path = find_model_path(args.model)
    if not model_path:
        simulation_app.close()
        return 1
    
    print(f"Model path: {model_path}")
    
    # Initialize World
    world = World()
    
    # Load humanoid USD model
    prim_path = "/World/Humanoid"
    print(f"Adding reference to stage at: {prim_path}")
    
    try:
        add_reference_to_stage(usd_path=model_path, prim_path=prim_path)
    except Exception as e:
        print(f"ERROR loading model: {e}")
        simulation_app.close()
        return 1
    
    # Verify the model was loaded
    stage = get_current_stage()
    humanoid_prim = stage.GetPrimAtPath(prim_path)
    
    if not humanoid_prim:
        print(f"ERROR: Humanoid not found at {prim_path}")
        simulation_app.close()
        return 1
    
    print(f"✓ Humanoid loaded successfully at {prim_path}")
    
    # Reset world to apply physics
    world.reset()
    print("✓ World reset - physics applied")
    
    # Verify physics properties for main body parts
    body_parts = ["torso", "head", "left_arm", "right_arm", "left_leg", "right_leg"]
    for part in body_parts:
        part_path = f"{prim_path}/{part}"
        verify_physics_properties(part_path)
    
    # List sensors
    sensors = list_sensors(prim_path)
    
    # Run simulation for 5 seconds if not headless
    if not args.headless:
        print("\nRunning simulation for 5 seconds...")
        print("Watch the humanoid fall under gravity.")
        for i in range(300):  # 60 Hz * 5 seconds
            world.step(render=True)
    
    print("\n✓ Model loading and verification complete!")
    
    # Cleanup
    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
