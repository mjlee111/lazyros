"""
ROS2 version compatibility module for lazyros.
"""

import os
import sys
from typing import Any, Optional, Union

# Import rclpy and related modules
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

# Try to import QoS-related modules with fallback
try:
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    QOS_AVAILABLE = True
except ImportError:
    QOS_AVAILABLE = False
    # Fallback for older versions
    try:
        from rclpy.qos import QoSProfile
        from rclpy.qos import ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        QOS_AVAILABLE = True
    except ImportError:
        QOS_AVAILABLE = False


def get_ros_distro() -> str:
    return os.environ.get("ROS_DISTRO", "unknown")


def is_ros2_humble() -> bool:
    return get_ros_distro() == "humble"


def is_ros2_jazzy() -> bool:
    return get_ros_distro() == "jazzy"


def is_ros2_iron() -> bool:
    return get_ros_distro() == "iron"


def is_ros2_galactic() -> bool:
    return get_ros_distro() == "galactic"


def is_ros2_foxy() -> bool:
    return get_ros_distro() == "foxy"


def is_supported_ros2() -> bool:
    supported_distros = ["humble", "jazzy", "iron", "galactic", "foxy", "rolling"]
    return get_ros_distro() in supported_distros


def create_qos_profile(depth: int = 10) -> Any:
    if not QOS_AVAILABLE:
        return None
    
    try:
        return QoSProfile(
            depth=depth,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )
    except Exception:
        try:
            return QoSProfile(
                depth=depth,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE
            )
        except Exception:
            return QoSProfile(depth=depth)


def create_callback_group() -> Any:
    try:
        return ReentrantCallbackGroup()
    except Exception:
        return None


class CompatibleRosRunner:
    def __init__(self) -> None:
        self._started = False
        self.executor: Optional[MultiThreadedExecutor] = None
        self.thread: Optional[Any] = None
        self.node: Optional[Node] = None

    def start(self) -> None:
        if self._started:
            return
        
        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            self.node = Node("lazyros_monitor_node")
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node)

            def _spin(executor: MultiThreadedExecutor):
                try:
                    executor.spin()
                except Exception:
                    pass

            self.thread = sys.modules['threading'].Thread(
                target=_spin, 
                args=(self.executor,), 
                daemon=False
            )
            self.thread.start()
            self._started = True
        except Exception as e:
            raise RuntimeError(f"Failed to start ROS runner: {e}")

    def stop(self) -> None:
        if not self._started:
            return
        
        try:
            if self.executor is not None:
                self.executor.shutdown()
            if self.thread is not None and self.thread.is_alive():
                self.thread.join(timeout=2.0)
            if self.executor is not None and self.node is not None:
                try:
                    self.executor.remove_node(self.node)
                except Exception:
                    pass
            if self.node is not None:
                try:
                    self.node.destroy_node()
                except Exception:
                    pass
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            self._started = False


def get_ros_info() -> str:
    ros_distro = get_ros_distro()
    ros_domain = os.environ.get("ROS_DOMAIN_ID", "0")
    dds_implementation = os.environ.get("RMW_IMPLEMENTATION", "unknown")
    return f"ROS_DISTRO={ros_distro}  |  ROS_DOMAIN_ID={ros_domain}  |  DDS_IMPLEMENTATION={dds_implementation}"


def check_ros_compatibility() -> bool:
    if not is_supported_ros2():
        ros_distro = get_ros_distro()
        print(f"Warning: Unsupported ROS2 distribution: {ros_distro}")
        print("Supported distributions: foxy, galactic, humble, iron, jazzy, rolling")
        return False
    return True