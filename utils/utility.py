import os
import threading
import yaml
import rclpy
from rclpy.node import Node

# Flag to signal the ROS thread to stop
shutdown_flag = threading.Event()

def ros_spin_thread(node: Node):
    """Function to spin the ROS node in a separate thread."""
    while rclpy.ok() and not shutdown_flag.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
    if hasattr(node, 'get_logger'): # Check if get_logger exists
        node.get_logger().info("ROS spin thread exiting.")
    else:
        print("ROS spin thread exiting (node logger not available).")


def load_restart_config(config_path="../config/restart_config.yaml"):
    """Load the restart configuration from a YAML file."""
    try:
        if not os.path.exists(config_path):
            print(f"Warning: Restart configuration file not found: {config_path}")
            return {}
            
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            
        #print(f"Loaded restart configuration for nodes: {list(config.get('nodes', {}).keys())}")
        return config
    except Exception as e:
        print(f"Error loading restart configuration: {e}")
        return {}

def signal_shutdown():
    """Signals the ROS thread to stop."""
    shutdown_flag.set()
