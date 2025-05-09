import os
import subprocess
from datetime import datetime
import asyncio

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal, VerticalScroll
from textual.widgets import (
    Label,
    ListItem,
    ListView,
    RichLog,
)
from rich.markup import escape# Import escape function


def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)  # Use the escape function from rich.markup

class NodeListWidget(Container):
    """A widget to display the list of ROS nodes."""

    # Add a binding for 'r' to restart the selected node
    BINDINGS = [
        Binding("r", "restart_node", "Restart Node"),
    ]

    def __init__(self, ros_node: Node, restart_config=None, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.node_list_view = ListView()
        self.previous_node_names = set() # Store previous names
        self.restart_config = restart_config or {} # Configuration for node restart commands
        print("NodeListWidget.__init__: Restart config:", self.restart_config)
        self.selected_node_name = None # Store the selected node name

    def compose(self) -> ComposeResult:
        yield Label("ROS Nodes:")
        yield VerticalScroll(self.node_list_view)

    def on_mount(self) -> None:
        """Called when the widget is mounted."""
        
        self.set_interval(1, self.update_node_list) # Update every second
        self.set_interval(0.1, self.update_log_and_info) # Update log and info every 0.1 seconds
        self.node_list_view.focus()

    def update_node_list(self) -> None:
        """Fetches and updates the list of ROS nodes, minimizing flicker."""
        try:
            node_names_and_namespaces = self.ros_node.get_node_names_and_namespaces()
            
            current_node_names = set()
            if node_names_and_namespaces:
                for name, namespace in node_names_and_namespaces:
                    if name.startswith("_") or name.startswith("launch_ros"):
                        continue
                    full_name = f"{namespace}/{name}" if namespace != "/" else f"/{name}"
                    current_node_names.add(full_name)

            # Only update the list view if the set of names has changed
            if current_node_names != self.previous_node_names:
                # Remember current selection
                current_index = self.node_list_view.index
                
                self.node_list_view.clear() # Clear existing items

                nodes = []
                # Sort names for consistent order (optional but good practice)
                sorted_names = sorted(list(current_node_names)) 
                if sorted_names:
                    for full_name in sorted_names:
                        nodes.append(ListItem(Label(full_name))) # Wrap string in Label
                else:
                     nodes.append(ListItem(Label("[No nodes found]")))

                self.node_list_view.extend(nodes)
                
                # Restore selection if possible
                if current_index is not None and current_index < len(nodes):
                     self.node_list_view.index = current_index
                elif len(nodes) > 0:
                    self.node_list_view.index = 0 # Select first item

                # Update the stored names
                self.previous_node_names = current_node_names

        except Exception as e:
            # Handle potential rclpy errors - display error only if it's new
            error_message = f"Error fetching nodes: {e}"
            if error_message not in self.previous_node_names: # Avoid constant error flicker
                 self.node_list_view.clear()
                 self.node_list_view.append(ListItem(Label(error_message)))
                 self.previous_node_names = {error_message} # Store the error message

    def action_restart_node(self) -> None:
        """Restart the currently selected node using the configuration."""
        print("NodeListWidget.action_restart_node: Method called")
        log_view = self.app.query_one("#log-view-content", LogViewWidget) # type: ignore
        log_view.rich_log.write("[bold yellow]Restart node action triggered[/]")
        
        # Get the currently selected node
        if self.node_list_view.index is None:
            print("NodeListWidget.action_restart_node: No node selected")
            log_view.rich_log.write("[red]No node selected[/]")
            return
        
        try:
            # Get the selected node name
            if self.node_list_view.index < 0 or self.node_list_view.index >= len(self.node_list_view.children):
                print("NodeListWidget.action_restart_node: Invalid selection index")
                log_view.rich_log.write("[red]Invalid selection index[/]")
                return
                
            selected_item = self.node_list_view.children[self.node_list_view.index]
            if not selected_item:
                print("NodeListWidget.action_restart_node: Cannot find selected node")
                log_view.rich_log.write("[red]Cannot find selected node[/]")
                return
                
            print(f"NodeListWidget.action_restart_node: Selected item: {selected_item}")
            print(f"NodeListWidget.action_restart_node: Selected item type: {type(selected_item)}")
            print(f"NodeListWidget.action_restart_node: Selected item children: {selected_item.children}")
            
            if not selected_item.children:
                print("NodeListWidget.action_restart_node: Selected item has no children")
                log_view.rich_log.write("[red]Selected item has no children[/]")
                return
                
            child = selected_item.children[0]
            print(f"NodeListWidget.action_restart_node: Child: {child}")
            print(f"NodeListWidget.action_restart_node: Child type: {type(child)}")
            
            node_name = str(child.renderable).strip() if hasattr(child, 'renderable') else str(child).strip() # type: ignore
            print(f"NodeListWidget.action_restart_node: Attempting to restart node: '{node_name}'")
            print(f"NodeListWidget.action_restart_node: Available nodes in config: {list(self.restart_config.get('nodes', {}).keys())}")
            log_view.rich_log.write(f"[bold]Attempting to restart node: '{node_name}'[/]")
            log_view.rich_log.write(f"Available nodes in config: {list(self.restart_config.get('nodes', {}).keys())}")
            
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
            
            # Find the process ID of the node and kill it
            try:
                # Use `pgrep` to find the process ID of the node
                pgrep_command = f"pgrep -f {node_name.strip('/')}"
                #print(f"NodeListWidget.action_restart_node: Finding process ID with command: {pgrep_command}")
                log_view.rich_log.write(f"[yellow]Finding process ID with command: {pgrep_command}[/]")
                process_id = subprocess.check_output(pgrep_command, shell=True, text=True).strip().split("\n")[-2]
                print(f"NodeListWidget.action_restart_node: Found process ID: {process_id}")
                log_view.rich_log.write(f"[yellow]Found process ID: {process_id}[/]")

                # Kill the process
                kill_command = f"kill -9 {process_id}"
                print(f"NodeListWidget.action_restart_node: Killing process with command: {kill_command}")
                subprocess.run(kill_command, shell=True, check=True)
                log_view.rich_log.write(f"[red]Killed process ID: {process_id}[/]")
            except subprocess.CalledProcessError as e:
                print(f"NodeListWidget.action_restart_node: Failed to find or kill process for {node_name}: {e}")
                log_view.rich_log.write(f"[red]Failed to find or kill process for {node_name}: {e}[/]")
                return
                
            command = node_config['command']
            print(f"NodeListWidget.action_restart_node: Executing restart command: {command}")
            log_view.rich_log.write(f"[green]Executing restart command: {command}[/]")
            
            subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            self.set_timer(2, self.update_node_list)
            
        except Exception as e:
            print(f"NodeListWidget.action_restart_node: Error restarting node: {e}")
            log_view.rich_log.write(f"[red]Error restarting node: {e}[/]")

    def on_list_view_highlighted(self, event): # type: ignore
        """Called when a node is highlighted/selected in the list."""
        try:
            selected_item = self.node_list_view.children[self.node_list_view.index] # type: ignore
            child = selected_item.children[0]
            raw_name = str(child.renderable).strip() if hasattr(child, 'renderable') else str(child).strip() # type: ignore
            node_name = raw_name[1:].replace('/', '.') if raw_name.startswith('/') else raw_name.replace('/', '.')
            self.selected_node_name = node_name
            print(f"NodeListWidget.on_list_view_highlighted: Selected node: {node_name}")
            
        except Exception as e:
            print(f"Error updating log filter: {e}")
            
    def update_log_and_info(self):
        """Update the log and info views based on the selected node."""
        if not self.selected_node_name:
            return

        try:
            log_view = self.app.query_one("#log-view-content", LogViewWidget) # type: ignore
            log_view.filter_logs(self.selected_node_name)
            info_view = self.app.query_one("#info-view-content", InfoViewWidget) # type: ignore
            info_view.update_info(self.selected_node_name)
        except Exception as e:
            print(f"Error updating log and info views: {e}")

class LogViewWidget(Container):
    """A widget to display ROS logs from /rosout."""

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.rich_log = RichLog(wrap=True, highlight=True, markup=True, max_lines=1000) 
        self.log_level_styles = {
            Log.DEBUG: "[dim cyan]",
            Log.INFO: "[dim blue]",
            Log.WARN: "[yellow]",
            Log.ERROR: "[bold red]",
            Log.FATAL: "[bold magenta]",
        }
        self.logs_by_node: dict[str, list[str]] = {}
        self.filtered_node: str | None = None

    def compose(self) -> ComposeResult:
        yield self.rich_log

    def on_mount(self) -> None:
        """Called when the widget is mounted. Create subscriber."""
        try:
            self.rich_log.write("[bold green]LogViewWidget mounted[/]")
            self.rich_log.write("[yellow]Testing logging system[/]")
            self.rich_log.write("[red]This is a test error message[/]")
            
            self.ros_node.create_subscription(
                Log,
                '/rosout',
                self.log_callback,
                10 # QoS depth
            )
            self.rich_log.write("[dim green]Log subscriber initialized.[/]")
        except Exception as e:
             self.rich_log.write(f"[bold red]Error creating /rosout subscriber: {e}[/]")

    def log_callback(self, msg: Log) -> None:
        """Callback for /rosout messages."""
        try:
            timestamp = datetime.fromtimestamp(msg.stamp.sec + msg.stamp.nanosec / 1e9)
            time_str = timestamp.strftime('%H:%M:%S.%f')[:-3] # Milliseconds
            level_style = self.log_level_styles.get(msg.level, "[dim white]") # Default style
            level_char = self._level_to_char(msg.level)
            escaped_msg = msg.msg.replace("[", "\\[")

            formatted_log = (
                f"{level_style}{time_str} "
                f"[{level_char}] "
                f"[{msg.name}] " # Node name
                f"{escaped_msg}[/]" # Apply style reset at the end
            )

            if msg.name not in self.logs_by_node:
                self.logs_by_node[msg.name] = []
            self.logs_by_node[msg.name].append(formatted_log)
            
            if not self.filtered_node or msg.name == self.filtered_node:
                self.app.call_from_thread(self.rich_log.write, formatted_log)
        except Exception as e:
            print(f"Error processing log message: {e}")

    def filter_logs(self, node_name: str | None = None):
        """Filter logs to show only those from the specified node."""
        self.filtered_node = node_name
        self.rich_log.clear()
        
        if not node_name:
            self.rich_log.write("[bold green]Showing logs for all nodes[/]")
            for _node, logs in self.logs_by_node.items():
                for log_entry in logs[-100:]:
                    self.rich_log.write(log_entry)
            return
            
        self.rich_log.write(f"[bold green]Showing logs for node: {node_name}[/]")
        if node_name in self.logs_by_node:
            for log_entry in self.logs_by_node[node_name][-200:]:
                self.rich_log.write(log_entry)
        else:
            self.rich_log.write(f"[yellow]No logs found for node: {node_name}[/]")

    def _level_to_char(self, level: int) -> str:
        """Convert log level integer to a single character representation."""
        if level == Log.DEBUG[0]: return "DEBUG"
        if level == Log.INFO[0]: return "INFO"
        if level == Log.WARN[0]: return "WARN"
        if level == Log.ERROR[0]: return "ERROR"
        if level == Log.FATAL[0]: return "FATAL"
        return "?"


class InfoViewWidget(Container):
    """Widget for displaying ROS node information."""

    DEFAULT_CSS = """
    InfoViewWidget {
        overflow-y: scroll; /* Make the container itself scrollable */
    }
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        # Replace Static with RichLog
        self.info_log = RichLog(wrap=True, highlight=True, markup=True, id="info-log", max_lines=1000) # Added max_lines
        self.info_dict = {} # Store node info

    def compose(self) -> ComposeResult:
        # Yield the RichLog widget
        yield self.info_log

    def update_info(self, node_name: str):
        """Update the displayed node information using `ros2 node info` output."""
        
        if node_name in self.info_dict:
            # If the info is already cached, use it
            formatted_lines = self.info_dict[node_name]
            self.info_log.clear()
            for line in formatted_lines:
                self.info_log.write(line)
            return
        # If not cached, fetch the info using subprocess
        print(f"InfoViewWidget.update_info: Fetching info for node: {node_name}")
        
        try:
            command = ["ros2", "node", "info", "/" + node_name]
            result = subprocess.run(command, capture_output=True, text=True, check=True)

            lines = result.stdout.splitlines()
            sections: dict[str, list[str]] = {
                "Subscribers": [],
                "Publishers": [],
                "Service Servers": [],
                "Service Clients": [],
                "Action Servers": [],
                "Action Clients": []
            }
            current_section = None

            for line in lines:
                line = line.strip()
                if line.endswith("Subscribers:"):
                    current_section = "Subscribers"
                    sections[current_section].append("Subscribers:")
                elif line.endswith("Publishers:"):
                    current_section = "Publishers"
                    sections[current_section].append("Publishers:")
                elif line.endswith("Service Servers:"):
                    current_section = "Service Servers"
                    sections[current_section].append("Service Servers:")
                elif line.endswith("Service Clients:"):
                    current_section = "Service Clients"
                    sections[current_section].append("Service Clients:")
                elif line.endswith("Action Servers:"):
                    current_section = "Action Servers"
                    sections[current_section].append("Action Servers:")
                elif line.endswith("Action Clients:"):
                    current_section = "Action Clients"
                    sections[current_section].append("Action Clients:")
                elif line.startswith("/") and current_section:
                    sections[current_section].append(line)

            formatted_lines = []
            for section_name, line_list in sections.items():
                formatted_lines.append("") # Separator line
                if (section_name == "Subscribers" or section_name == "Publishers"):
                    for stripped_line in line_list: 
                        if stripped_line.startswith("/"):
                            topic, *rest = stripped_line.split(":", 1)
                            topic_display = topic
                            topic_arg = topic.replace("'", "\\'").replace('"', '\\"')
                            rest_of_line = escape_markup(rest[0]) if rest else ""
                            formatted_lines.append(
                                f"[blue][@click=app.handle_topic_click('{topic_arg}')]{topic_display}[/][/blue]:"
                                f"[green][@click=app.handle_message_click('{rest_of_line}')]{rest_of_line}[/][/green]"
                            )
                        else:
                            formatted_lines.append(f"[bold]{escape_markup(stripped_line)}[/bold]")              
                else:
                    for stripped_line in line_list:
                        if stripped_line.startswith("/"):
                            formatted_lines.append(escape_markup(stripped_line))
                        else:
                            formatted_lines.append(f"[bold]{escape_markup(stripped_line)}[/bold]")

            # Clear previous content and write new lines to RichLog
            self.info_log.clear()
            for line in formatted_lines:
                self.info_log.write(line)
            # Cache the info for future use
            self.info_dict[node_name] = formatted_lines

        except FileNotFoundError:
            self.info_log.clear()
            self.info_log.write("[red]ros2 command not found. Please ensure ROS 2 is installed and sourced.[/]")
        except subprocess.TimeoutExpired:
            self.info_log.clear()
            self.info_log.write("[red]Timeout expired while fetching node info.[/]")

        except subprocess.CalledProcessError as e:
            self.info_log.clear() # Clear before writing error
            self.info_log.write(f"[red]Error fetching info for node: /{node_name}[/]\n\n{escape_markup(e.stderr)}")
        except Exception as e:
            self.info_log.clear() # Clear before writing error
            self.info_log.write(f"[red]Unexpected error fetching info for /{node_name}: {escape_markup(str(e))}[/red]")
