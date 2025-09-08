import subprocess

from rclpy.node import Node
from rclpy.action import graph
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import RichLog
from rich.markup import escape
from textual.widgets import Static 
from rich.text import Text
import asyncio


def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

class InfoViewWidget(Container):
    """Widget for displaying ROS node information."""

    DEFAULT_CSS = """
    InfoViewWidget {
        overflow-y: scroll;
    }
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node # May not be directly used if info comes from subprocess
        self.rich_log = RichLog(wrap=True, highlight=True, markup=True, id="info-log", max_lines=1000)
        self.info_dict: dict[str, list[str]] = {} # Cache for node info
        
        self.selected_node_data = None
        self.current_node_full_name = None

    def compose(self) -> ComposeResult:
        yield Static("", id="node-info")
        
    def on_mount(self) -> None:
        self.set_interval(1, self.update_info)  # Update info every 0.5 seconds
            
    async def update_info(self):
        node_listview = self.app.query_one("#node-listview")
        
        view = self.query_one("#node-info", Static)
        if not node_listview.selected_node_name:
            view.update("[red]No node is selected yet.[/]")
            return
        self.selected_node_data = node_listview.node_listview_dict["/"+node_listview.selected_node_name]

        if self.selected_node_data is None:
            view.update("[red]No node is selected yet.[/]")
            return

        if self.selected_node_data.status != "green":
            view.update("[red]Selected node is shutdown.[/]")
            return

        if self.selected_node_data.full_name == self.current_node_full_name:
            return
        
        self.current_node_full_name = self.selected_node_data.full_name
        info_lines = await asyncio.to_thread(self.show_node_info)
        if info_lines:
            view.update(Text.from_markup("\n".join(info_lines)))


    def show_node_info(self) -> None:
        node_data = self.selected_node_data
        if node_data.full_name in self.info_dict:
            return self.info_dict[node_data.full_name]

        full_name = node_data.full_name 
        node = self.selected_node_data.node_name
        namespace = self.selected_node_data.namespace
        
        pubs = self.ros_node.get_publisher_names_and_types_by_node(node, namespace)
        subs = self.ros_node.get_subscriber_names_and_types_by_node(node, namespace)
        service_servers = self.ros_node.get_service_names_and_types_by_node(node, namespace)
        service_clients = self.ros_node.get_client_names_and_types_by_node(node, namespace)
        action_servers = graph.get_action_server_names_and_types_by_node(self.ros_node, node, namespace) 
        action_clients = graph.get_action_client_names_and_types_by_node(self.ros_node, node, namespace) 

        info_lines = []
        info_lines.append(f"{full_name}") 
        info_lines.append(f"  Subscribers:")
        for sub in subs:
            topic = sub[0]
            type_list = sub[1]
            info_lines.append(f"      {escape_markup(topic)}: {escape_markup(', '.join(type_list))}")
        info_lines.append(f"  Publishers:")
        for pub in pubs:
            topic = pub[0]
            type_list = pub[1]
            info_lines.append(f"      {escape_markup(topic)}: {escape_markup(', '.join(type_list))}")
        info_lines.append(f"  Service Clients:")
        for client in service_clients:
            service = client[0]
            type_list = client[1]
            info_lines.append(f"      {escape_markup(service)}: {escape_markup(', '.join(type_list))}")
        info_lines.append(f"  Service Servers:")
        for server in service_servers:
            service = server[0]
            type_list = server[1]
            info_lines.append(f"      {escape_markup(service)}: {escape_markup(', '.join(type_list))}")
        info_lines.append(f"  Action Servers:")
        for server in action_servers:
            action = server[0]
            type_list = server[1]
            info_lines.append(f"      {escape_markup(action)}: {escape_markup(', '.join(type_list))}")
        info_lines.append(f"  Action Clients:")
        for client in action_clients:
            action = client[0]
            type_list = client[1]
            info_lines.append(f"      {escape_markup(action)}: {escape_markup(', '.join(type_list))}")
        
        self.info_dict[full_name] = info_lines
        return info_lines
