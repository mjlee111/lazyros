import subprocess
import re

from rclpy.node import Node
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import RichLog
from rich.markup import escape

def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

class ParameterValueWidget(Container):
    """Widget for displaying ROS parameter values."""

    DEFAULT_CSS = """
    ParameterValueWidget {
        overflow-y: scroll;
    }
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.value_log = RichLog(wrap=True, highlight=True, markup=True, id="parameter-value-log", max_lines=1000)
        self.current_parameter = None
        self.ros_node.create_timer(0.5, self.display_parameter_value)

    def compose(self) -> ComposeResult:
        yield self.value_log

    def update_parameter(self, parameter_text: str):
        self.current_parameter = parameter_text

    def display_parameter_value(self):
        """Display the value of a parameter using `ros2 param get` command."""
        
        try:
            match = re.fullmatch(r"([^:]+):\s*(.+)", self.current_parameter)
            if not match:
                self.value_log.clear()
                self.value_log.write(f"[red]Invalid parameter format: {escape_markup(self.current_parameter)}[/]")
                return
                
            node_name = match.group(1).strip()
            param_name = match.group(2).strip()
            
            command = ["ros2", "param", "get", node_name, param_name]
            result = subprocess.run(command, capture_output=True, text=True, check=True, timeout=5)
            
            self.value_log.clear()
            self.value_log.write(f"[bold]Parameter Value: {escape_markup(param_name)}[/bold]")
            self.value_log.write(f"[bold]Node: {escape_markup(node_name)}[/bold]")
            self.value_log.write("")
            
            for line in result.stdout.splitlines():
                self.value_log.write(escape_markup(line))
            
        except FileNotFoundError:
            self.value_log.clear()
            self.value_log.write("[red]ros2 command not found. Ensure ROS 2 is installed and sourced.[/]")
        except subprocess.TimeoutExpired:
            self.value_log.clear()
            self.value_log.write(f"[red]Timeout fetching value for parameter: {escape_markup(self.current_parameter)}[/]")
        except subprocess.CalledProcessError as e:
            self.value_log.clear()
            self.value_log.write(f"[red]Error fetching parameter value: {escape_markup(e.stderr or str(e))}[/]")
        except Exception as e:
            self.value_log.clear()
            self.value_log.write(f"[red]Error fetching parameter value: {escape_markup(str(e))}[/]")
    
    def clear_log(self):
        """Clear the parameter value log."""

        self.value_log.clear()
        if self.current_parameter:
            self.value_log.write(f"[bold]Parameter: {escape_markup(self.current_parameter)}[/bold]")
            self.value_log.write("")