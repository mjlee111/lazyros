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


class ParameterInfoWidget(Container):
    """Widget for displaying ROS parameter information."""

    DEFAULT_CSS = """
    ParameterInfoWidget {
        overflow-y: scroll;
    }
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)

        self.ros_node = ros_node
        self.info_log = RichLog(wrap=True, highlight=True, markup=True, id="parameter-info-log", max_lines=1000)
        self.current_parameter = None

        self.ros_node.create_timer(0.1, self.display_parameter_info)

    def compose(self) -> ComposeResult:
        yield self.info_log

    #def on_mount(self) -> None:
    #    self.set_interval(0.5, self.display_parameter_info)

    def display_parameter_info(self):
        """Display the information of a parameter using `ros2 param describe` command."""
        
        if self.current_parameter is None:
            self.info_log.clear()
            self.info_log.write("[red]No parameter is selected yet.[/]")   

        try:
            match = re.fullmatch(r"([^:]+):\s*(.+)", self.current_parameter)
            if not match:
                self.info_log.clear()
                self.info_log.write(f"[red]Invalid parameter format: {escape_markup(self.current_parameter)}[/]")
                return
                
            node_name = match.group(1).strip()
            param_name = match.group(2).strip()
            
            command = ["ros2", "param", "describe", node_name, param_name]
            result = subprocess.run(command, capture_output=True, text=True, check=True, timeout=5)
            
            self.info_log.clear()
            self.info_log.write(f"[bold]Parameter Information: {escape_markup(param_name)}[/bold]")
            self.info_log.write(f"[bold]Node: {escape_markup(node_name)}[/bold]")
            self.info_log.write("")
            
            for line in result.stdout.splitlines():
                self.info_log.write(escape_markup(line))
            
        except FileNotFoundError:
            self.info_log.clear()
            self.info_log.write("[red]ros2 command not found. Ensure ROS 2 is installed and sourced.[/]")
        except subprocess.TimeoutExpired:
            self.info_log.clear()
            self.info_log.write(f"[red]Timeout fetching description for parameter: {escape_markup(self.current_parameter)}[/]")
        except subprocess.CalledProcessError as e:
            self.info_log.clear()
            self.info_log.write(f"[red]Error fetching parameter description: {escape_markup(e.stderr or str(e))}[/]")
        except Exception as e:
            self.info_log.clear()
            self.info_log.write(f"[red]Error fetching parameter description: {escape_markup(str(e))}[/]")
    
    def clear_log(self):
        """Clear the parameter info log."""
        self.info_log.clear()
        if self.current_parameter:
            self.info_log.write(f"[bold]Parameter: {escape_markup(self.current_parameter)}[/bold]")
            self.info_log.write("")