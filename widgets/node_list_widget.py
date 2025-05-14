import os
import subprocess
from datetime import datetime # Not used directly by NodeListWidget but often useful
import asyncio # Not used directly by NodeListWidget but often useful

import rclpy # Not used directly by NodeListWidget but often useful
from rclpy.node import Node
from textual.app import App, ComposeResult # App might not be needed directly
from textual.binding import Binding
from textual.containers import Container, VerticalScroll, ScrollableContainer, HorizontalScroll
from textual.widgets import (
    Label,
    ListItem,
    ListView,
)
from rich.markup import escape # Import escape function
from rich.text import Text as RichText # Import RichText for combining elements

from utils.ignore_parser import IgnoreParser # Import IgnoreParser
from dataclasses import dataclass


def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

@dataclass
class NodeData:
    """A data class to hold information about a ROS node."""
    name: str
    status: str

class NodeListWidget(Container):
    """A widget to display the list of ROS nodes."""

    BINDINGS = [
        Binding("r", "restart_node", "Restart Node"),
    ]

    def __init__(self, ros_node: Node, restart_config=None, ignore_file_path='config/display_ignore.yaml', **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.node_list_view = ListView()
        self.previous_node_names = set()
        self.restart_config = restart_config or {}
        self.selected_node_name = None
        self.ignore_parser = IgnoreParser(ignore_file_path) # Instantiate IgnoreParser
        
        self.launched_nodes = {}

    def compose(self) -> ComposeResult:
        yield Label("ROS Nodes:")
        yield self.node_list_view

    def on_mount(self) -> None:
        self.set_interval(1, self.update_node_list)
        self.set_interval(1, self.update_log_and_info)
        self.node_list_view.focus()

    def update_node_list(self) -> None:        

        node_set = set(self.launched_nodes.keys())
        nodes = []
        need_update = False

        try:
            command = "ros2 node list"
            result = subprocess.run(command, shell=True, capture_output=True, text=True)
            if result.returncode != 0:
                nodes.append(ListItem(Label("[red]Error fetching nodes[/]")))
                return

            for line in result.stdout.splitlines():
                node_name = line.strip()
                if not self.ignore_parser.should_ignore(node_name, 'node'):
                    node_raw_name = node_name[1:] if node_name.startswith("/") else node_name
                    node_slash_name = f"/{node_raw_name}"

                    if node_raw_name not in self.launched_nodes:
                        self.launched_nodes[node_raw_name] = NodeData(name=node_raw_name, status="green")
                        need_update = True

                    else:
                        if self.launched_nodes[node_raw_name].status != "green":
                            self.launched_nodes[node_raw_name].status = "green"
                            need_update = True

                        node_set.discard(node_raw_name)

            for i in node_set:
                if self.launched_nodes[i].status != "red":
                    self.launched_nodes[i].status = "red"
                    need_update = True

            if not need_update:
                return    

            launched_nodes = self.launched_nodes.keys()
            sorted_names = sorted(list(launched_nodes))

            current_index = self.node_list_view.index 
            self.node_list_view.clear()
            for i in sorted_names:
                status_indicator = RichText("●", style=f"bold {self.launched_nodes[i].status}")
                node_label = RichText(i)
                combined_label = RichText.assemble(status_indicator, "  ", node_label)
                nodes.append(ListItem(Label(combined_label)))

            self.node_list_view.extend(nodes)

            if current_index is not None and current_index < len(nodes):
                self.node_list_view.index = current_index
            elif len(nodes) > 0:
                self.node_list_view.index = 0

                     
        except Exception as e:
            error_message = f"Error fetching nodes: {e}"
            if error_message not in self.previous_node_names:
                 self.node_list_view.clear()
                 self.node_list_view.append(ListItem(Label(error_message)))

    def action_restart_node(self) -> None:
        print("NodeListWidget.action_restart_node: Method called")
        # Assuming LogViewWidget is accessible via app query
        log_view = self.app.query_one("#log-view-content") # type: ignore 
        log_view.rich_log.write("[bold yellow]Restart node action triggered[/]")
        
        if self.node_list_view.index is None:
            print("NodeListWidget.action_restart_node: No node selected")
            log_view.rich_log.write("[red]No node selected[/]")
            return
        
        try:
            if self.node_list_view.index < 0 or self.node_list_view.index >= len(self.node_list_view.children):
                print("NodeListWidget.action_restart_node: Invalid selection index")
                log_view.rich_log.write("[red]Invalid selection index[/]")
                return
                
            selected_item = self.node_list_view.children[self.node_list_view.index]
            if not selected_item or not selected_item.children:
                print("NodeListWidget.action_restart_node: Cannot find selected node or selected item has no children")
                log_view.rich_log.write("[red]Cannot find selected node or selected item has no children[/]")
                return
                
            child = selected_item.children[0]
            node_name = str(child.renderable).strip() if hasattr(child, 'renderable') else str(child).strip()
            print(f"NodeListWidget.action_restart_node: Attempting to restart node: '{node_name}'")
            log_view.rich_log.write(f"[bold]Attempting to restart node: '{node_name}'[/]")
            
            if node_name.startswith("[") and node_name.endswith("]"):
                print("NodeListWidget.action_restart_node: Cannot restart: not a valid node")
                log_view.rich_log.write("[red]Cannot restart: not a valid node[/]")
                return
                
            if not self.restart_config or 'nodes' not in self.restart_config:
                print(f"NodeListWidget.action_restart_node: No restart configuration available for {node_name}")
                log_view.rich_log.write(f"[red]No restart configuration available for {node_name}[/]")
                return
                
            node_config = self.restart_config['nodes'].get(node_name)
            if not node_config or 'command' not in node_config:
                print(f"NodeListWidget.action_restart_node: No restart command configured for {node_name}")
                log_view.rich_log.write(f"[red]No restart command configured for {node_name}[/]")
                return
            
            try:
                pgrep_command = f"pgrep -f {node_name.strip('/')}"
                log_view.rich_log.write(f"[yellow]Finding process ID with command: {pgrep_command}[/]")
                process_id_output = subprocess.check_output(pgrep_command, shell=True, text=True).strip()
                # pgrep might return multiple PIDs, often the actual process is not the last one.
                # A more robust way might be needed, but for now, let's try to find one that's not the pgrep itself.
                # This is a simplification.
                process_ids = [pid for pid in process_id_output.split("\n") if pid]
                if not process_ids:
                    raise subprocess.CalledProcessError(1, pgrep_command, "No process ID found")
                process_id = process_ids[0] # Taking the first one, might need refinement
                
                print(f"NodeListWidget.action_restart_node: Found process ID: {process_id}")
                log_view.rich_log.write(f"[yellow]Found process ID: {process_id}[/]")

                kill_command = f"kill -9 {process_id}"
                print(f"NodeListWidget.action_restart_node: Killing process with command: {kill_command}")
                subprocess.run(kill_command, shell=True, check=True)
                log_view.rich_log.write(f"[red]Killed process ID: {process_id}[/]")
            except subprocess.CalledProcessError as e:
                print(f"NodeListWidget.action_restart_node: Failed to find or kill process for {node_name}: {e}")
                log_view.rich_log.write(f"[red]Failed to find or kill process for {node_name}: {e.stderr if e.stderr else e.stdout}[/]")
                return
                
            command = node_config['command']
            print(f"NodeListWidget.action_restart_node: Executing restart command: {command}")
            log_view.rich_log.write(f"[green]Executing restart command: {command}[/]")
            
            subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            self.set_timer(2, self.update_node_list)
            
        except Exception as e:
            print(f"NodeListWidget.action_restart_node: Error restarting node: {e}")
            log_view.rich_log.write(f"[red]Error restarting node: {e}[/]")

    def on_list_view_highlighted(self, event): # type: ignore
        try:
            if self.node_list_view.index is None or self.node_list_view.index < 0 or self.node_list_view.index >= len(self.node_list_view.children):
                self.selected_node_name = None
                return

            selected_item = self.node_list_view.children[self.node_list_view.index]
            if not selected_item.children:
                self.selected_node_name = None
                return

            child = selected_item.children[0]
            raw_name = str(child.renderable).strip() if hasattr(child, 'renderable') else str(child).strip()
            
            # Assuming node names from get_node_names_and_namespaces are like /node_name or /namespace/node_name
            # For ros2 node info, it expects just "node_name" or "namespace/node_name" (without leading /)
            # For log filtering, it seems the full name including leading / is used in msg.name
            # Let's adjust self.selected_node_name to be without leading / for `ros2 node info`
            # but keep in mind that logs might use the full path.
            # The current log filtering uses msg.name which is full path.
            # The current info view update uses "/" + node_name.
            # Let's make selected_node_name the one used for `ros2 node info` (no leading /)
            
            if raw_name.startswith('/'):
                self.selected_node_name = raw_name[1:] 
            else:
                self.selected_node_name = raw_name

            # For log filtering, we might need the full name if msg.name includes it.
            # The current LogViewWidget.filter_logs expects the name as it appears in msg.name.
            # The current NodeListWidget.update_log_and_info passes self.selected_node_name.
            # This needs to be consistent. Let's assume msg.name is the full path.
            # And NodeListWidget stores the full path for filtering.
            # So, self.selected_node_name should be the full path.
            self.selected_node_name = raw_name # Store the full name as displayed

            print(f"NodeListWidget.on_list_view_highlighted: Selected node: {self.selected_node_name}")
            
        except Exception as e:
            print(f"Error in on_list_view_highlighted: {e}")
            self.selected_node_name = None
            
    def update_log_and_info(self):
        if not self.selected_node_name or self.selected_node_name.startswith("["): # Don't update for placeholder messages
            return

        try:
            info_node_name = self.selected_node_name[1:] if self.selected_node_name.startswith('/') else self.selected_node_name
            log_filter_name = self.selected_node_name.strip("●  ") # Full path for log filtering

            log_view = self.app.query_one("#log-view-content") # type: ignore
            log_view.filter_logs(log_filter_name.replace("/", ".")) # Pass the full path for filtering
            
            info_view = self.app.query_one("#info-view-content") # type: ignore
            info_view.update_info("/"+log_filter_name)
        except Exception as e:
            print(f"Error updating log and info views from NodeListWidget: {e}")
