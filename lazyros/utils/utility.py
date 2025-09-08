import re
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

def create_css_id(name: str) -> str:
    """
    Clean and convert a string to a valid CSS identifier.
    identifiers must contain only letters, numbers, underscores, or hyphens, and must not begin with a number.
    """
    css_id = re.sub(r'[^a-zA-Z0-9_-]', '_', name)
    css_id = css_id.lower()

    if css_id and css_id[0].isdigit():
        css_id = "_" + css_id 

    return css_id 


class RosRunner:
    def __init__(self) -> None:
        self._started = False
        self.executor: MultiThreadedExecutor | None = None
        self.thread: threading.Thread | None = None
        self.node: Node | None = None

    def start(self) -> None:
        if self._started:
            return
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

        self.thread = threading.Thread(target=_spin, args=(self.executor,), daemon=False)
        self.thread.start()
        self._started = True

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