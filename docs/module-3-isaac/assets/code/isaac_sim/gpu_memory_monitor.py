#!/usr/bin/env python3
"""
Monitor GPU memory usage during Isaac Sim simulation.

This script provides real-time monitoring of GPU VRAM usage and warns
when approaching memory limits. Useful for optimizing scene complexity.

Usage:
    python gpu_memory_monitor.py --threshold 0.9 --interval 1.0

Requirements:
    - pynvml (NVIDIA Management Library Python bindings)
    - Install: pip install nvidia-ml-py3
"""

import argparse
import sys
import time
from datetime import datetime

try:
    import pynvml
except ImportError:
    print("ERROR: pynvml not installed!")
    print("Install with: pip install nvidia-ml-py3")
    sys.exit(1)


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="GPU memory monitor for Isaac Sim")
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.9,
        help="Warning threshold (0.0-1.0, default: 0.9 = 90%%)"
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Sampling interval in seconds (default: 1.0)"
    )
    parser.add_argument(
        "--log",
        type=str,
        default=None,
        help="Optional: Log to file (e.g., gpu_log.csv)"
    )
    return parser.parse_args()


def format_bytes(bytes):
    """Format bytes to human-readable string."""
    for unit in ['B', 'KB', 'MB', 'GB']:
        if bytes < 1024.0:
            return f"{bytes:.2f} {unit}"
        bytes /= 1024.0
    return f"{bytes:.2f} TB"


def get_gpu_info():
    """
    Get GPU memory usage information.
    
    Returns:
        dict: GPU information including memory usage
    """
    pynvml.nvmlInit()
    
    device_count = pynvml.nvmlDeviceGetCount()
    if device_count == 0:
        raise RuntimeError("No NVIDIA GPUs found!")
    
    # Get info for first GPU (most Isaac Sim setups use single GPU)
    handle = pynvml.nvmlDeviceGetHandleByIndex(0)
    
    info = {
        "name": pynvml.nvmlDeviceGetName(handle),
        "driver_version": pynvml.nvmlSystemGetDriverVersion(),
        "memory_info": pynvml.nvmlDeviceGetMemoryInfo(handle),
        "utilization": pynvml.nvmlDeviceGetUtilizationRates(handle),
        "temperature": pynvml.nvmlDeviceGetTemperature(handle, pynvml.NVML_TEMPERATURE_GPU),
    }
    
    # Calculate memory usage percentage
    mem_info = info["memory_info"]
    info["memory_percent"] = mem_info.used / mem_info.total
    
    return info


def print_header():
    """Print monitoring header."""
    print("=" * 80)
    print("GPU Memory Monitor for Isaac Sim")
    print("=" * 80)
    print("Press Ctrl+C to stop monitoring\n")


def print_gpu_status(info, threshold, warning_issued):
    """
    Print current GPU status.
    
    Returns:
        bool: True if warning was issued this update
    """
    timestamp = datetime.now().strftime("%H:%M:%S")
    mem_info = info["memory_info"]
    mem_percent = info["memory_percent"]
    
    # Color codes
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    RESET = '\033[0m'
    
    # Determine color based on usage
    if mem_percent < threshold * 0.8:
        color = GREEN
        status = "OK"
    elif mem_percent < threshold:
        color = YELLOW
        status = "HIGH"
    else:
        color = RED
        status = "CRITICAL"
    
    print(f"[{timestamp}] {color}GPU Memory: {format_bytes(mem_info.used)} / "
          f"{format_bytes(mem_info.total)} ({mem_percent * 100:.1f}%) | "
          f"GPU Util: {info['utilization'].gpu}% | "
          f"Temp: {info['temperature']}°C | "
          f"Status: {status}{RESET}")
    
    # Issue warning if threshold exceeded
    new_warning = False
    if mem_percent >= threshold and not warning_issued:
        print(f"\n{RED}⚠️  WARNING: GPU memory usage exceeds {threshold * 100:.0f}% threshold!{RESET}")
        print(f"{RED}   Consider:")
        print(f"{RED}   - Reducing scene complexity (fewer objects, lower resolution){RESET}")
        print(f"{RED}   - Closing other GPU applications{RESET}")
        print(f"{RED}   - Using a GPU with more VRAM{RESET}\n")
        new_warning = True
    
    return new_warning


def main():
    """Main function."""
    args = parse_args()
    
    # Initialize NVML
    try:
        pynvml.nvmlInit()
    except Exception as e:
        print(f"ERROR initializing NVML: {e}")
        print("Make sure NVIDIA drivers are installed correctly.")
        return 1
    
    # Get initial GPU info
    try:
        initial_info = get_gpu_info()
        print_header()
        print(f"GPU: {initial_info['name']}")
        print(f"Driver Version: {initial_info['driver_version']}")
        print(f"Total VRAM: {format_bytes(initial_info['memory_info'].total)}")
        print(f"Warning Threshold: {args.threshold * 100:.0f}%")
        print(f"Sampling Interval: {args.interval}s")
        if args.log:
            print(f"Logging to: {args.log}")
        print("-" * 80 + "\n")
    except Exception as e:
        print(f"ERROR getting GPU info: {e}")
        pynvml.nvmlShutdown()
        return 1
    
    # Setup logging if requested
    log_file = None
    if args.log:
        log_file = open(args.log, 'w')
        log_file.write("timestamp,used_mb,total_mb,percent,gpu_util,temperature\n")
    
    # Monitoring loop
    warning_issued = False
    try:
        while True:
            # Get current GPU info
            info = get_gpu_info()
            
            # Print status
            if print_gpu_status(info, args.threshold, warning_issued):
                warning_issued = True
            
            # Log to file if requested
            if log_file:
                timestamp = datetime.now().isoformat()
                mem_info = info["memory_info"]
                log_file.write(f"{timestamp},"
                              f"{mem_info.used / (1024**2):.2f},"
                              f"{mem_info.total / (1024**2):.2f},"
                              f"{info['memory_percent'] * 100:.2f},"
                              f"{info['utilization'].gpu},"
                              f"{info['temperature']}\n")
                log_file.flush()
            
            # Wait before next sample
            time.sleep(args.interval)
    
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped by user.")
    except Exception as e:
        print(f"\nERROR during monitoring: {e}")
        return 1
    finally:
        if log_file:
            log_file.close()
            print(f"Log saved to: {args.log}")
        pynvml.nvmlShutdown()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
