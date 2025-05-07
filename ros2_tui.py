import os
import subprocess
from datetime import datetime
import asyncio  # Add asyncio for asynchronous processing

import rclpy
import yaml
from rcl_interfaces.msg import Log
from rclpy.node import Node
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal, VerticalScroll
from textual.widgets import (
    Footer,
    Header,
    Label,
    ListItem,
    ListView,
    RichLog,
    Static,
    TabbedContent,  # Import TabbedContent
    Markdown,       # Import Markdown
    Button,         # Import Button
)
from textual.screen import ModalScreen
from rich.color import Color
from rich.markup import escape as escape_markup # Import escape function
# Removed hashlib import


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

    def compose(self) -> ComposeResult:
        yield Label("ROS Nodes:")
        yield VerticalScroll(self.node_list_view)

    def on_mount(self) -> None:
        """Called when the widget is mounted."""
        print("NodeListWidget.on_mount: Widget mounted")
        self.update_node_list()
        self.set_interval(1.0, self.update_node_list) # Update every second
        # Set focus to the list view
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
        log_view = self.app.query_one("#log-view-content")
        log_view.rich_log.write("[bold yellow]Restart node action triggered[/]")
        
        # Get the currently selected node
        if self.node_list_view.index is None:
            print("NodeListWidget.action_restart_node: No node selected")
            log_view.rich_log.write("[red]No node selected[/]")
            return
            
        try:
            # Get the selected node name
            # Instead of get_child_at_index, use the proper way to access the selected item
            if self.node_list_view.index < 0 or self.node_list_view.index >= len(self.node_list_view.children):
                print("NodeListWidget.action_restart_node: Invalid selection index")
                log_view.rich_log.write("[red]Invalid selection index[/]")
                return
                
            selected_item = self.node_list_view.children[self.node_list_view.index]
            if not selected_item:
                print("NodeListWidget.action_restart_node: Cannot find selected node")
                log_view.rich_log.write("[red]Cannot find selected node[/]")
                return
                
            # Extract the node name from the ListItem
            # First, check the structure to make sure we're accessing it correctly
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
            
            # Get the text content safely by converting to string
            node_name = str(child.renderable).strip() if hasattr(child, 'renderable') else str(child).strip()
            print(f"NodeListWidget.action_restart_node: Attempting to restart node: '{node_name}'")
            print(f"NodeListWidget.action_restart_node: Available nodes in config: {list(self.restart_config.get('nodes', {}).keys())}")
            log_view.rich_log.write(f"[bold]Attempting to restart node: '{node_name}'[/]")
            log_view.rich_log.write(f"Available nodes in config: {list(self.restart_config.get('nodes', {}).keys())}")
            
            # Check if this is an error message rather than a node name
            if node_name.startswith("[") and node_name.endswith("]"):
                print("NodeListWidget.action_restart_node: Cannot restart: not a valid node")
                log_view.rich_log.write("[red]Cannot restart: not a valid node[/]")
                return
                
            # Look up the restart command in the configuration
            if not self.restart_config or 'nodes' not in self.restart_config:
                print(f"NodeListWidget.action_restart_node: No restart configuration available for {node_name}")
                log_view.rich_log.write(f"[red]No restart configuration available for {node_name}[/]")
                return
                
            node_config = self.restart_config['nodes'].get(node_name)
            if not node_config or 'command' not in node_config:
                print(f"NodeListWidget.action_restart_node: No restart command configured for {node_name}")
                log_view.rich_log.write(f"[red]No restart command configured for {node_name}[/]")
                return
                
            # Get the command to restart the node
            command = node_config['command']
            print(f"NodeListWidget.action_restart_node: Executing restart command: {command}")
            log_view.rich_log.write(f"[green]Executing restart command: {command}[/]")
            
            # Execute the command in a separate process
            # Use subprocess.Popen to avoid blocking the UI
            subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # Update the node list after a short delay to reflect changes
            self.set_timer(2, self.update_node_list)
            
        except Exception as e:
            print(f"NodeListWidget.action_restart_node: Error restarting node: {e}")
            log_view.rich_log.write(f"[red]Error restarting node: {e}[/]")

    def on_list_view_highlighted(self, event):
        """Called when a node is highlighted/selected in the list."""
        try:
            selected_item = self.node_list_view.children[self.node_list_view.index]
            child = selected_item.children[0]
            # Get the node name and normalize it:
            # 1. Get raw name from renderable or string
            # 2. Remove leading slash if present
            # 3. Replace remaining slashes with dots
            raw_name = str(child.renderable).strip() if hasattr(child, 'renderable') else str(child).strip()
            node_name = raw_name[1:].replace('/', '.') if raw_name.startswith('/') else raw_name.replace('/', '.')
            
            # Update log filter
            log_view = self.app.query_one("#log-view-content")
            log_view.filter_logs(node_name)

            # Update node info
            info_view = self.app.query_one("#info-view-content")
            info_view.update_info(node_name)
            
        except Exception as e:
            print(f"Error updating log filter: {e}")


class LogViewWidget(Container):
    """A widget to display ROS logs from /rosout."""

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        # Use max_lines to prevent performance issues with very long logs
        self.rich_log = RichLog(wrap=True, highlight=True, markup=True, max_lines=1000) 
        # Mapping from ROS log levels to Rich markup styles
        self.log_level_styles = {
            Log.DEBUG: "[dim cyan]",
            Log.INFO: "[dim blue]",
            Log.WARN: "[yellow]",
            Log.ERROR: "[bold red]",
            Log.FATAL: "[bold magenta]",
        }
        # Store logs by node name for filtering
        self.logs_by_node = {}
        # Current node filter
        self.filtered_node = None

    def compose(self) -> ComposeResult:
        yield self.rich_log

    def on_mount(self) -> None:
        """Called when the widget is mounted. Create subscriber."""
        try:
            # Add test messages to verify logging is working
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
        # Format the log message
        try:
            timestamp = datetime.fromtimestamp(msg.stamp.sec + msg.stamp.nanosec / 1e9)
            time_str = timestamp.strftime('%H:%M:%S.%f')[:-3] # Milliseconds
            level_style = self.log_level_styles.get(msg.level, "[dim white]") # Default style
            level_char = self._level_to_char(msg.level)

            # Escape Rich markup in the message itself to prevent interference
            escaped_msg = msg.msg.replace("[", "\\[")

            formatted_log = (
                f"{level_style}{time_str} "
                f"[{level_char}] "
                f"[{msg.name}] " # Node name
                f"{escaped_msg}[/]" # Apply style reset at the end
            )

            # Store log by node name
            if msg.name not in self.logs_by_node:
                self.logs_by_node[msg.name] = []
            self.logs_by_node[msg.name].append(formatted_log)
            
            # Only display if no filter is set or if the log matches the current filter
            if not self.filtered_node or msg.name == self.filtered_node:
                # Safely update the RichLog from the ROS thread
                self.app.call_from_thread(self.rich_log.write, formatted_log)
        except Exception as e:
            # Log errors happening within the callback itself to the TUI log
            # Avoid calling self.rich_log.write here to prevent potential infinite loops
            print(f"Error processing log message: {e}")

    def filter_logs(self, node_name=None):
        """Filter logs to show only those from the specified node."""
        self.filtered_node = node_name
        
        # Clear existing logs
        self.rich_log.clear()
        
        # If no filter, show all logs
        if not node_name:
            self.rich_log.write("[bold green]Showing logs for all nodes[/]")
            for node, logs in self.logs_by_node.items():
                for log in logs[-100:]:  # Show last 100 logs per node
                    self.rich_log.write(log)
            return
            
        # Show logs only for the selected node
        self.rich_log.write(f"[bold green]Showing logs for node: {node_name}[/]")
        if node_name in self.logs_by_node:
            for log in self.logs_by_node[node_name][-200:]:  # Show last 200 logs
                self.rich_log.write(log)
        else:
            self.rich_log.write(f"[yellow]No logs found for node: {node_name}[/]")

    def _level_to_char(self, level: int) -> str:
        """Convert log level integer to a single character representation."""
        if level == Log.DEBUG: return "D"
        if level == Log.INFO: return "I"
        if level == Log.WARN: return "W"
        if level == Log.ERROR: return "E"
        if level == Log.FATAL: return "F"
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
        # Revert to using a single Static widget for info display
        self.info_static = Static("Node Information will appear here.", id="info-text", markup=True)

    def compose(self) -> ComposeResult:
        # Display the Static widget directly
        yield self.info_static # REMOVED VerticalScroll wrapper

    def update_info(self, node_name: str):
        """Update the displayed node information using `ros2 node info` output."""
        try:
            command = ["ros2", "node", "info", "/" + node_name]
            result = subprocess.run(command, capture_output=True, text=True, check=True)

            lines = result.stdout.splitlines()
            sections = {
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

                # セクション切り替え
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
            
            for section, line_list in sections.items():
                if (section == "Subscribers" or section == "Publishers"):
                    for stripped_line in line_list: 
                        if stripped_line.startswith("/"):
                            topic = stripped_line.split(":")[0]
                            escaped_topic_display = escape_markup(topic)
                            escaped_topic_arg = topic.replace("'", "\\'").replace('"', '\\"')
                            rest_of_line = escape_markup(stripped_line.split(':', 1)[1])
                            formatted_lines.append(
                                f"[blue][@click=app.handle_topic_click('{escaped_topic_arg}')]{escaped_topic_display}[/][/blue]:{rest_of_line}"
                            )
                        else:
                            formatted_lines.append(f"[bold]{escape_markup(stripped_line)}[/bold]")
                else:
                    for stripped_line in line_list:
                        if stripped_line.startswith("/"):
                            formatted_lines.append(escape_markup(stripped_line))
                        else:
                            formatted_lines.append(f"[bold]{escape_markup(stripped_line)}[/bold]")

            self.info_static.update("\n".join(formatted_lines))

        except subprocess.CalledProcessError as e:
            self.info_static.update(f"[red]Error fetching info for node: /{node_name}[/]\n\n{escape_markup(e.stderr)}")
        except Exception as e:
            self.info_static.update(f"[red]Unexpected error fetching info for /{node_name}: {escape_markup(str(e))}[/red]")


class RosTuiApp(App):
    """A Textual app to monitor ROS information."""

    BINDINGS = [
        ("d", "toggle_dark", "Toggle dark mode"),
        ("q", "quit", "Quit"),
        ("escape", "quit", "Quit"),
        ("r", "restart_node", "Restart Node"),
        ("shift+left", "focus_left_pane", "Focus Left Pane"),
        ("shift+right", "focus_right_pane", "Focus Right Pane"),
    ]

    CSS_PATH = "ros2_tui.css"

    def __init__(self, ros_node: Node, restart_config=None):
        super().__init__()
        self.ros_node = ros_node
        self.restart_config = restart_config

    async def async_setup(self):
        """Perform asynchronous setup tasks to speed up initialization."""
        # Use asyncio.create_task to avoid blocking the event loop
        asyncio.create_task(self.preload_node_list())
        asyncio.create_task(self.preload_logs())

    async def preload_node_list(self):
        """Preload the node list to reduce startup delay."""
        try:
            # Fetch node names asynchronously without blocking
            node_names_and_namespaces = await asyncio.to_thread(self.ros_node.get_node_names_and_namespaces)
            print("Preloaded node list:", node_names_and_namespaces)
        except Exception as e:
            print("Error preloading node list:", e)

    async def preload_logs(self):
        """Preload logs to reduce startup delay."""
        try:
            # Simulate log preloading asynchronously
            await asyncio.sleep(0.1)  # Replace with actual log fetching logic
            print("Preloaded logs.")
        except Exception as e:
            print("Error preloading logs:", e)

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        print("RosTuiApp.compose: Composing the application layout...")
        yield Header()

        with Horizontal():
            # Left pane with frame and title
            with Container(id="left-frame", classes="left-pane"):
                print("Adding left pane...")
                yield Static("Nodes", classes="frame-title")
                yield NodeListWidget(self.ros_node, self.restart_config, id="node-list-content")

            # Right pane with frame and title
            with Container(id="right-frame", classes="right-pane"):
                print("Adding right pane...")
                yield Static("Logs and Info", classes="frame-title")
                with TabbedContent("Log", "Info"):
                    yield LogViewWidget(self.ros_node, id="log-view-content")
                    yield InfoViewWidget(self.ros_node, id="info-view-content")

        yield Footer()
        print("Application layout composed.")

    def action_toggle_dark(self) -> None:
        """An action to toggle dark mode."""
        self.dark = not self.dark
        
    def action_restart_node(self) -> None:
        """Forward restart_node action to the NodeListWidget."""
        print("RosTuiApp.action_restart_node: Forwarding action to NodeListWidget")
        node_list = self.query_one("#node-list-content")
        if node_list:
            node_list.action_restart_node()

    def action_focus_left_pane(self) -> None:
        """Focus the left pane and highlight it."""
        left_pane: Container = self.query_one("#left-frame")
        right_pane: Container = self.query_one("#right-frame")
    
        # Add a bold border to the left pane and remove it from the right pane
        left_pane.styles.border = ("heavy", "white")
        right_pane.styles.border = ("solid", "white")
    
        # Focus the left pane
        left_pane.focus()
    
    def action_focus_right_pane(self) -> None:
        """Focus the right pane and highlight it."""
        left_pane: Container = self.query_one("#left-frame")
        right_pane: Container = self.query_one("#right-frame")
    
        left_pane.styles.border = ("solid", "white")
        right_pane.styles.border = ("heavy", "white")

    
        # Focus the right pane
        right_pane.focus()

    def action_handle_topic_click(self, topic_name: str) -> None:
        """Handle clicks on topic 'links' in the InfoViewWidget."""
        print(f"Topic clicked: {topic_name}")
        # Display a simple modal dialog
        self.push_screen(TopicInfoModal(topic_name))

    def run(self):
        """Override run to include async setup."""
        asyncio.run(self.async_setup())
        super().run()

# Define a simple modal screen for displaying topic info
class TopicInfoModal(ModalScreen):
    """A modal screen to display topic information."""

    CSS = """
    TopicInfoModal {
        align: center middle;
        layer: modal; /* Ensure it appears above the main screen */
    }

    #modal-container {
        width: 50%;
        height: 50%;
        border: round white;
        background: $background;
        align: center middle;
    }

    #modal-message {
        margin: 1 0;
        text-align: center;
    }

    #modal-instruction {
        margin-top: 1;
        text-align: center;
    }
    """

    BINDINGS = [
        ("q", "dismiss", "Quit Modal")
    ]

    def __init__(self, topic_name: str, **kwargs):
        super().__init__(**kwargs)
        self.topic_name = topic_name
        self.topic_info = self.get_topic_info()

    def get_topic_info(self) -> str:
        """Fetch the topic information using `ros2 topic info` command."""
        try:
            result = subprocess.run(
                ["ros2", "topic", "info", self.topic_name],
                capture_output=True,
                text=True,
                check=True
            )
            return result.stdout
        except subprocess.CalledProcessError as e:
            return f"Error fetching topic info: {e.stderr.strip()}"
        except Exception as e:
            return f"Unexpected error: {str(e)}"

    def compose(self) -> ComposeResult:
        """Compose the modal dialog."""
        yield Container(
            Label(f"Topic Information: {self.topic_name}", id="modal-message"),
            Label(self.topic_info, id="modal-content"),
            Label("Press 'q' to quit.", id="modal-instruction"),
            id="modal-container",
            classes="modal-content",
        )


import threading  # Add threading import here

# Flag to signal the ROS thread to stop
shutdown_flag = threading.Event()

def ros_spin_thread(node: Node):
    """Function to spin the ROS node in a separate thread."""
    while rclpy.ok() and not shutdown_flag.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info("ROS spin thread exiting.")


def load_restart_config(config_path="restart_config.yaml"):
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


def main(args=None):
    rclpy.init(args=args)
    ros_node = None
    app = None
    ros_thread = None
    try:
        # Load the restart configuration
        restart_config = load_restart_config()
        #print(f"Main: loaded restart configuration with keys: {restart_config.keys()}")
        
        # Create a generic node for introspection and logging
        # Using a unique name to avoid conflicts
        ros_node = Node("ros2_tui_monitor_node")

        # Start the ROS spinning in a separate thread
        ros_thread = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
        ros_thread.start()

        app = RosTuiApp(ros_node, restart_config)
        app.run() # This blocks until the app exits

    except Exception as e:
        print(f"Error initializing ROS or running the TUI: {e}")
    finally:
        # Signal the ROS thread to stop
        shutdown_flag.set()
        
        # Wait briefly for the ROS thread to finish
        if ros_thread:
            ros_thread.join(timeout=1.0)

        # Destroy the node and shutdown rclpy
        if ros_node:
            ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
        print("ROS TUI exited cleanly.")


if __name__ == "__main__":
    main()
