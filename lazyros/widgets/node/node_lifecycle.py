import subprocess

from rclpy.node import Node
from rclpy.action import graph
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import RichLog
from rich.markup import escape
from dataclasses import dataclass

import rclpy
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.srv import ChangeState 
from lifecycle_msgs.msg import State as LifecycleState
from rclpy.callback_groups import ReentrantCallbackGroup


def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

@dataclass
class LifecycleData:
    is_lifecycle: bool
    get_lifecycle_client: callable
    change_lifecycle_client: callable


class LifecycleWidget(Container):
    """Widget for displaying ROS node information."""

    DEFAULT_CSS = """
    InfoViewWidget {
        overflow-y: scroll;
    }
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node # May not be directly used if info comes from subprocess
        self.info_log = RichLog(wrap=True, highlight=True, markup=True, id="info-log", max_lines=1000)
        self.lifecycle_dict: dict[str, list[str]] = {} # Cache for node info
        
        self.selected_node_data = None
        self.current_node_full_name = None

    def compose(self) -> ComposeResult:
        yield self.info_log
        
    def on_mount(self) -> None:
        self.set_interval(0.5, self.update_display)  # Update info every 0.5 seconds

    def update_display(self):
        # No node selected, clear display
        if self.selected_node_data is None:
            self.info_log.clear()
            self.info_log.write("[red]No node is selected yet.[/]")
            return

        # Node is the same, no need to update
        if self.selected_node_data.full_name == self.current_node_full_name:
            return
               
        self.info_log.clear()
        self.current_node_full_name = self.selected_node_data.full_name
        if self.selected_node_data.full_name not in self.lifecycle_dict:
            self.create_lifecycle_data()
        
        if not self.lifecycle_dict[self.selected_node_data.full_name].is_lifecycle:
            return self.info_log.write(f"[red]Node {self.selected_node_data.full_name} is not a lifecycle node.[/]")
        
        info_lines = self.get_lifecycle_state()
        self.info_log.write("\n".join(info_lines))


    def create_lifecycle_data(self) -> bool:
        node = self.selected_node_data.node_name
        namespace = self.selected_node_data.namespace

        is_lifecycle = False
        srvs = self.ros_node.get_service_names_and_types_by_node(node, namespace)
        for srv in srvs:
            if "lifecycle_msgs/srv/GetState" in srv[1]:
                is_lifecycle = True
                break
            
        if is_lifecycle:
            self.lifecycle_dict[self.selected_node_data.full_name] = \
                LifecycleData(is_lifecycle=True,
                            get_lifecycle_client=self.ros_node.create_client(GetState, f"{self.selected_node_data.full_name}/get_state", callback_group=ReentrantCallbackGroup()),
                            change_lifecycle_client=self.ros_node.create_client(ChangeState, f"{self.selected_node_data.full_name}/change_state"))
        else:
            self.lifecycle_dict[self.selected_node_data.full_name] = \
                LifecycleData(is_lifecycle=False,
                            get_lifecycle_client=None,
                            change_lifecycle_client=None)

    def get_lifecycle_state(self) -> None:
        """If the node is a lifecycle node, get its state."""
        
        full_name = self.selected_node_data.full_name
        
        lifecycle_client = self.lifecycle_dict[full_name].get_lifecycle_client
        if not lifecycle_client.wait_for_service(timeout_sec=1.0):
            return f"[red]Lifecycle service for {full_name} is not available[/]"

        req = GetState.Request()
        future = lifecycle_client.call_async(req)
        rclpy.spin_until_future_complete(self.ros_node, future)
        if not future.done() or future.result() is None:
            return f"[red]Failed to get lifecycle state for {full_name}[/]"

        current = future.result().current_state
        info_lines = []
        info_lines.append(f"Lifecycle State for {full_name}:")
        info_lines.append(f"  {current.label}[{current.id}]")
        return info_lines


        