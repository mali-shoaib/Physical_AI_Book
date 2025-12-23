#!/usr/bin/env python3
"""
Capture RGB images and depth maps from Isaac Sim stereo cameras.

This script demonstrates how to programmatically capture synthetic sensor data
from Isaac Sim cameras and save it to disk.

Usage:
    python capture_rgb_depth.py --num-frames 50 --output ./dataset

Requirements:
    - Isaac Sim 2024.1+
    - PIL (Pillow) for image saving
    - NumPy for depth processing
"""

import argparse
import sys
from pathlib import Path
import numpy as np
from PIL import Image

from omni.isaac.kit import SimulationApp

# Parse arguments before launching
parser = argparse.ArgumentParser(description="Capture RGB and depth data")
parser.add_argument("--num-frames", type=int, default=50, help="Number of frames to capture")
parser.add_argument("--output", type=str, default="./synthetic_dataset", help="Output directory")
parser.add_argument("--resolution", type=str, default="1280x720", help="Camera resolution (WxH)")
parser.add_argument("--headless", action="store_true", help="Run without GUI")
args = parser.parse_args()

# Parse resolution
width, height = map(int, args.resolution.split('x'))

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": args.headless})

# Import after launch
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.render_product import create_hydra_texture
import omni.replicator.core as rep


def setup_cameras(world):
    """Load humanoid and get camera render products."""
    print("Loading humanoid with stereo cameras...")
    
    # Load model
    model_path = str(Path(__file__).parent.parent.parent / "models" / "humanoid_stereo_cameras.usda")
    add_reference_to_stage(usd_path=model_path, prim_path="/World/Humanoid")
    
    world.reset()
    
    # Create render products for cameras
    print(f"Creating render products ({width}x{height})...")
    
    left_camera_path = "/World/Humanoid/head/left_camera"
    right_camera_path = "/World/Humanoid/head/right_camera"
    
    # Use replicator for rendering
    left_rp = rep.create.render_product(left_camera_path, (width, height))
    right_rp = rep.create.render_product(right_camera_path, (width, height))
    
    return left_rp, right_rp


def capture_frame(render_product, frame_id, output_dir, camera_side):
    """
    Capture RGB and depth from a render product.
    
    Args:
        render_product: Isaac Sim render product
        frame_id (int): Frame number
        output_dir (Path): Output directory
        camera_side (str): 'left' or 'right'
    """
    # Capture RGB
    rgb_data = rep.AnnotatorRegistry.get_annotator("rgb", device="cuda")
    rgb_data.attach([render_product])
    
    rgb_array = rgb_data.get_data()
    
    if rgb_array is not None and len(rgb_array) > 0:
        # Convert to uint8 and save
        rgb_img = Image.fromarray(rgb_array.astype(np.uint8), mode='RGB')
        rgb_path = output_dir / f"{camera_side}_rgb_{frame_id:04d}.png"
        rgb_img.save(rgb_path)
    
    # Capture depth
    depth_data = rep.AnnotatorRegistry.get_annotator("distance_to_camera", device="cuda")
    depth_data.attach([render_product])
    
    depth_array = depth_data.get_data()
    
    if depth_array is not None and len(depth_array) > 0:
        # Save raw depth as NumPy
        depth_path = output_dir / f"{camera_side}_depth_{frame_id:04d}.npy"
        np.save(depth_path, depth_array)
        
        # Also save visualized depth as PNG
        depth_vis = ((depth_array / depth_array.max()) * 255).astype(np.uint8)
        depth_img = Image.fromarray(depth_vis, mode='L')
        depth_vis_path = output_dir / f"{camera_side}_depth_{frame_id:04d}_vis.png"
        depth_img.save(depth_vis_path)
    
    return rgb_path, depth_path


def main():
    """Main function."""
    # Create output directory
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Output directory: {output_dir}")
    print(f"Capturing {args.num_frames} frames...")
    
    # Initialize World
    world = World()
    
    # Setup cameras
    try:
        left_rp, right_rp = setup_cameras(world)
    except Exception as e:
        print(f"ERROR setting up cameras: {e}")
        simulation_app.close()
        return 1
    
    print("\nStarting capture...")
    print("="*60)
    
    # Capture loop
    for frame_id in range(args.num_frames):
        # Step simulation
        world.step(render=True)
        
        # Capture from both cameras
        try:
            left_rgb, left_depth = capture_frame(left_rp, frame_id, output_dir, "left")
            right_rgb, right_depth = capture_frame(right_rp, frame_id, output_dir, "right")
            
            print(f"Frame {frame_id + 1}/{args.num_frames}: "
                  f"RGB saved ({left_rgb.stat().st_size // 1024}KB left, "
                  f"{right_rgb.stat().st_size // 1024}KB right)")
        
        except Exception as e:
            print(f"ERROR capturing frame {frame_id}: {e}")
            continue
    
    print("="*60)
    print(f"\n✓ Dataset captured successfully!")
    print(f"  Total frames: {args.num_frames}")
    print(f"  Output: {output_dir.resolve()}")
    print(f"\nFiles per frame:")
    print(f"  - left_rgb_XXXX.png")
    print(f"  - left_depth_XXXX.npy")
    print(f"  - left_depth_XXXX_vis.png")
    print(f"  - right_rgb_XXXX.png")
    print(f"  - right_depth_XXXX.npy")
    print(f"  - right_depth_XXXX_vis.png")
    
    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
