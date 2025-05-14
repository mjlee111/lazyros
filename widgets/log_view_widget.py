from datetime import datetime

from rcl_interfaces.msg import Log
from rclpy.node import Node
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import RichLog
from rich.markup import escape
from rclpy.callback_groups import ReentrantCallbackGroup

def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

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
        self.callback_group = ReentrantCallbackGroup()  # Use a reentrant callback group for the subscription 

    def compose(self) -> ComposeResult:
        yield self.rich_log

    def on_mount(self) -> None:
        try:
            self.ros_node.create_subscription(
                Log,
                '/rosout',
                self.log_callback,
                10,
            )
        except Exception as e:
             self.rich_log.write(f"[bold red]Error creating /rosout subscriber: {e}[/]")

    def log_callback(self, msg: Log) -> None:
        try:
            timestamp = datetime.fromtimestamp(msg.stamp.sec + msg.stamp.nanosec / 1e9)
            time_str = timestamp.strftime('%H:%M:%S.%f')[:-3]
            level_style = self.log_level_styles.get(msg.level, "[dim white]")
            level_char = self._level_to_char(msg.level)
            
            escaped_msg_content = str(msg.msg).replace("[", "\\[")

            formatted_log = (
                f"{level_style}{time_str} "
                f"[{level_char}] "
                f"[{msg.name}] " 
                f"{escaped_msg_content}[/]"
            )

            if msg.name not in self.logs_by_node:
                self.logs_by_node[msg.name] = []
            self.logs_by_node[msg.name].append(formatted_log)
            
            # Filtered_node should be the full path like /node_name
            if not self.filtered_node or msg.name == self.filtered_node:
                self.app.call_from_thread(self.rich_log.write, formatted_log)
        except Exception as e:
            # Avoid printing directly to console from a callback if possible
            # Consider logging to a file or a dedicated debug area in the TUI
            print(f"Error processing log message in LogViewWidget: {e}")

    def filter_logs(self, node_name: str | None = None):
        """Filter logs to show only those from the specified node.
        node_name is expected to be the full path, e.g., /talker or /namespace/nodename
        """
        self.filtered_node = node_name # Store the full path
        self.rich_log.clear()
        
        if not node_name: # Show all logs if node_name is None or empty
            self.rich_log.write("[bold green]Showing logs for all nodes[/]")
            for _node_key, logs in self.logs_by_node.items():
                for log_entry in logs[-100:]: # Show last 100 from each node when unfiltered
                    self.rich_log.write(log_entry)
            return
            
        self.rich_log.write(f"[bold green]Showing logs for node: {node_name}[/]")
        if node_name in self.logs_by_node:
            for log_entry in self.logs_by_node[node_name][-200:]: # Show last 200 for filtered node
                self.rich_log.write(log_entry)
        else:
            self.rich_log.write(f"[yellow]No logs found for node: {node_name}[/]")

    def _level_to_char(self, level: int) -> str:
        if level == Log.DEBUG[0]: return "DEBUG" # Compare with Log.DEBUG directly
        if level == Log.INFO[0]: return "INFO"
        if level == Log.WARN[0]: return "WARN"
        if level == Log.ERROR[0]: return "ERROR"
        if level == Log.FATAL[0]: return "FATAL"
        return "?"
