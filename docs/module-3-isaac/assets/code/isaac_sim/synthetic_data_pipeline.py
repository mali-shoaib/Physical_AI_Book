#!/usr/bin/env python3
"""
Automated synthetic data generation pipeline with domain randomization.

This script demonstrates best practices for generating large-scale synthetic
datasets with randomized lighting, camera poses, and robot poses.

Usage:
    python synthetic_data_pipeline.py --num-frames 1000 --randomize

Requirements:
    - Isaac Sim 2024.1+
    - Sufficient disk space (~ 500MB per 100 frames)
"""

import argparse
import sys
import json
from pathlib import Path
import numpy as np
from PIL import Image
import time

from omni.isaac.kit import SimulationApp

# Parse arguments
parser = argparse.ArgumentParser(description="Synthetic data generation pipeline")
parser.add_argument("--num-frames", type=int, default=100, help="Number of frames to generate")
parser.add_argument("--output", type=str, default="./synthetic_data", help="Output directory")
parser.add_argument("--randomize", action="store_true", help="Enable domain randomization")
parser.add_argument("--segmentation", action="store_true", help="Include semantic segmentation")
parser.add_argument("--headless", action="store_true", help="Run without GUI")
args = parser.parse_args()

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": args.headless})

# Import after launch
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.replicator.core as rep
from pxr import Gf


def setup_scene(world):
    """Setup the scene with humanoid and environment."""
    print("Setting up scene...")
    
    # Load humanoid
    model_path = str(Path(__file__).parent.parent.parent / "models" / "humanoid_stereo_cameras.usda")
    add_reference_to_stage(usd_path=model_path, prim_path="/World/Humanoid")
    
    # Add ground plane
    world.scene.add_default_ground_plane()
    
    world.reset()
    
    # Create render products
    left_camera = "/World/Humanoid/head/left_camera"
    rp = rep.create.render_product(left_camera, (1280, 720))
    
    return rp


def randomize_scene():
    """Apply domain randomization to lighting and robot pose."""
    if not args.randomize:
        return
    
    # Randomize lighting
    light_intensity = np.random.uniform(500, 2000)
    rep.modify.attribute("/World/defaultLight", "intensity", light_intensity)
    
    # Randomize ground color
    ground_color = Gf.Vec3f(
        np.random.uniform(0.3, 0.8),
        np.random.uniform(0.3, 0.8),
        np.random.uniform(0.3, 0.8)
    )
    rep.modify.attribute("/World/groundPlane", "primvars:displayColor", [ground_color])
    
    # Randomize humanoid position slightly
    x_offset = np.random.uniform(-0.5, 0.5)
    y_offset = np.random.uniform(-0.5, 0.5)
    rep.modify.pose("/World/Humanoid", position=(x_offset, y_offset, 1.0))


def capture_annotated_frame(rp, frame_id, output_dir):
    """
    Capture RGB, depth, and optionally segmentation.
    
    Returns:
        dict: Metadata about captured frame
    """
    metadata = {
        "frame_id": frame_id,
        "timestamp": time.time(),
        "files": {}
    }
    
    # RGB
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb", device="cuda")
    rgb_annotator.attach([rp])
    rgb_data = rgb_annotator.get_data()
    
    if rgb_data is not None:
        rgb_img = Image.fromarray(rgb_data.astype(np.uint8), mode='RGB')
        rgb_path = f"rgb_{frame_id:06d}.png"
        rgb_img.save(output_dir / rgb_path)
        metadata["files"]["rgb"] = rgb_path
    
    # Depth
    depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_camera", device="cuda")
    depth_annotator.attach([rp])
    depth_data = depth_annotator.get_data()
    
    if depth_data is not None:
        depth_path = f"depth_{frame_id:06d}.npy"
        np.save(output_dir / depth_path, depth_data)
        metadata["files"]["depth"] = depth_path
        
        # Visualized depth
        depth_vis = ((depth_data / np.max(depth_data)) * 255).astype(np.uint8)
        depth_img = Image.fromarray(depth_vis, mode='L')
        depth_vis_path = f"depth_{frame_id:06d}_vis.png"
        depth_img.save(output_dir / depth_vis_path)
        metadata["files"]["depth_vis"] = depth_vis_path
    
    # Semantic segmentation (optional)
    if args.segmentation:
        seg_annotator = rep.AnnotatorRegistry.get_annotator("semantic_segmentation", device="cuda")
        seg_annotator.attach([rp])
        seg_data = seg_annotator.get_data()
        
        if seg_data is not None:
            seg_path = f"segmentation_{frame_id:06d}.npy"
            np.save(output_dir / seg_path, seg_data)
            metadata["files"]["segmentation"] = seg_path
    
    return metadata


def main():
    """Main function."""
    # Create output directory
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"{'='*70}")
    print(f"Synthetic Data Generation Pipeline")
    print(f"{'='*70}")
    print(f"Output: {output_dir}")
    print(f"Frames: {args.num_frames}")
    print(f"Randomization: {'Enabled' if args.randomize else 'Disabled'}")
    print(f"Segmentation: {'Enabled' if args.segmentation else 'Disabled'}")
    print(f"{'='*70}\n")
    
    # Initialize World
    world = World()
    
    # Setup scene
    try:
        rp = setup_scene(world)
    except Exception as e:
        print(f"ERROR setting up scene: {e}")
        simulation_app.close()
        return 1
    
    # Collect metadata for all frames
    dataset_metadata = {
        "num_frames": args.num_frames,
        "resolution": [1280, 720],
        "randomization": args.randomize,
        "segmentation": args.segmentation,
        "frames": []
    }
    
    # Generation loop
    start_time = time.time()
    print("Starting generation...")
    
    for frame_id in range(args.num_frames):
        # Apply randomization
        randomize_scene()
        
        # Step simulation
        world.step(render=True)
        
        # Capture frame
        try:
            frame_metadata = capture_annotated_frame(rp, frame_id, output_dir)
            dataset_metadata["frames"].append(frame_metadata)
            
            # Progress indicator
            if (frame_id + 1) % 10 == 0:
                elapsed = time.time() - start_time
                fps = (frame_id + 1) / elapsed
                eta = (args.num_frames - frame_id - 1) / fps if fps > 0 else 0
                print(f"Progress: {frame_id + 1}/{args.num_frames} "
                      f"({(frame_id + 1) / args.num_frames * 100:.1f}%) | "
                      f"FPS: {fps:.2f} | ETA: {eta:.0f}s")
        
        except Exception as e:
            print(f"ERROR on frame {frame_id}: {e}")
            continue
    
    # Save dataset metadata
    metadata_path = output_dir / "dataset_metadata.json"
    with open(metadata_path, 'w') as f:
        json.dump(dataset_metadata, f, indent=2)
    
    elapsed = time.time() - start_time
    
    print(f"\n{'='*70}")
    print(f"✓ Dataset generation complete!")
    print(f"{'='*70}")
    print(f"Total frames: {args.num_frames}")
    print(f"Total time: {elapsed:.1f}s ({args.num_frames / elapsed:.2f} FPS)")
    print(f"Output: {output_dir.resolve()}")
    print(f"Metadata: {metadata_path}")
    
    # Calculate dataset size
    total_size = sum(f.stat().st_size for f in output_dir.glob('*') if f.is_file())
    print(f"Dataset size: {total_size / (1024**2):.1f} MB")
    
    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
