import os
import subprocess
from datetime import datetime # Not used directly by NodeListWidget but often useful
import asyncio # Not used directly by NodeListWidget but often useful

import rclpy # Not used directly by NodeListWidget but often useful
from rclpy.node import Node
from textual.app import App, ComposeResult # App might not be needed directly
from textual.binding import Binding
from textual.containers import Container, VerticalScroll, ScrollableContainer
from textual.widgets import (
    Label,
    ListItem,
    ListView,
)
from rich.markup import escape # Import escape function

from utils.ignore_parser import IgnoreParser # Import IgnoreParser

# Assuming LogViewWidget and InfoViewWidget will be in their own files
# and imported if direct interaction is needed (currently interaction is via app.query_one)
# from .log_view_widget import LogViewWidget # Example if needed
# from .info_view_widget import InfoViewWidget # Example if needed

def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

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
        print("NodeListWidget.__init__: Restart config:", self.restart_config)
        self.selected_node_name = None
        self.ignore_parser = IgnoreParser(ignore_file_path) # Instantiate IgnoreParser

    def compose(self) -> ComposeResult:
        yield Label("ROS Nodes:")
        yield ScrollableContainer(self.node_list_view)

    def on_mount(self) -> None:
        self.set_interval(1, self.update_node_list)
        self.set_interval(0.1, self.update_log_and_info)
        self.node_list_view.focus()

    def update_node_list(self) -> None:
        try:
            node_names_and_namespaces = self.ros_node.get_node_names_and_namespaces()
            
            current_node_names = set()
            if node_names_and_namespaces:
                for name, namespace in node_names_and_namespaces:
                    if name.startswith("_") or name.startswith("launch_ros"):
                        continue
                    full_name = f"{namespace}/{name}" if namespace != "/" else f"/{name}"
                    # Filter nodes based on the ignore list
                    if not self.ignore_parser.should_ignore(full_name, 'node'):
                        current_node_names.add(full_name)

            if current_node_names != self.previous_node_names:
                current_index = self.node_list_view.index
                self.node_list_view.clear()
                nodes = []
                sorted_names = sorted(list(current_node_names))
                if sorted_names:
                    for full_name in sorted_names:
                        nodes.append(ListItem(Label(full_name)))
                else:
                     nodes.append(ListItem(Label("[No nodes found]")))
                self.node_list_view.extend(nodes)
                
                if current_index is not None and current_index < len(nodes):
                     self.node_list_view.index = current_index
                elif len(nodes) > 0:
                    self.node_list_view.index = 0
                self.previous_node_names = current_node_names
        except Exception as e:
            error_message = f"Error fetching nodes: {e}"
            if error_message not in self.previous_node_names:
                 self.node_list_view.clear()
                 self.node_list_view.append(ListItem(Label(error_message)))
                 self.previous_node_names = {error_message}

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
            # Pass the name as it's used by ros2 node info (without leading /)
            # and as used by log messages (full path)
            # The current InfoViewWidget.update_info prepends "/" if it's not there.
            # The current LogViewWidget.filter_logs uses the name directly.
            # Let's ensure consistency. If selected_node_name is full path like "/talker":
            #  - InfoViewWidget needs "talker"
            #  - LogViewWidget needs "/talker" (if msg.name is "/talker")
            
            info_node_name = self.selected_node_name[1:] if self.selected_node_name.startswith('/') else self.selected_node_name
            log_filter_name = self.selected_node_name # Full path for log filtering

            log_view = self.app.query_one("#log-view-content") # type: ignore
            log_view.filter_logs(log_filter_name.strip("/")) # Pass the full path for filtering
            
            info_view = self.app.query_one("#info-view-content") # type: ignore
            info_view.update_info(log_filter_name)
        except Exception as e:
            print(f"Error updating log and info views from NodeListWidget: {e}")
