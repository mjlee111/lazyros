import asyncio
import re
from typing import Any, Dict, List, Optional

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import DescribeParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rich.markup import escape
from rich.text import Text
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import RichLog, Static



PARAMETER_TYPE_MAP = {
    ParameterType.PARAMETER_BOOL: "bool",
    ParameterType.PARAMETER_INTEGER: "integer",
    ParameterType.PARAMETER_DOUBLE: "double",
    ParameterType.PARAMETER_STRING: "string",
    ParameterType.PARAMETER_BYTE_ARRAY: "byte_array",
    ParameterType.PARAMETER_BOOL_ARRAY: "bool_array",
    ParameterType.PARAMETER_INTEGER_ARRAY: "integer_array",
    ParameterType.PARAMETER_DOUBLE_ARRAY: "double_array",
    ParameterType.PARAMETER_STRING_ARRAY: "string_array",
}

class ParameterInfoWidget(Container):
    """Widget for displaying ROS parameter information."""

    DEFAULT_CSS = """
        ParameterInfoWidget {
            overflow-y: scroll;
        }
    """

    def __init__(self, ros_node: Node, **kwargs: Any) -> None:
        """Initialize the ParameterInfoWidget.
        
        Args:
            ros_node: The ROS node instance for communication
            **kwargs: Additional keyword arguments passed to the parent Container
        """
        super().__init__(**kwargs)

        self.ros_node = ros_node
        self.rich_log = RichLog(wrap=True, highlight=True, markup=True, id="parameter-info-log", max_lines=1000)
        self.param_client_dict = {}
        self.listview_widget = None

        self.current_parameter = None
        self.select_parameter = None

    def compose(self) -> ComposeResult:
        """Compose the widget layout.
        
        Returns:
            ComposeResult: A generator yielding widget components
        """
        yield Static("", id="parameter-info")

    def on_mount(self) -> None:
        """Handle the widget mount event.
        
        Sets up periodic interval to update display.
        """
        self.set_interval(1, self.update_display)

    async def update_display(self) -> None:
        """Update the parameter information display.
        
        Shows parameter details for the currently selected parameter.
        Updates are skipped if no parameter is selected.
        """
        
        self.listview_widget = self.app.query_one("#parameter-listview")
        self.selected_parameter = self.listview_widget.selected_param if self.listview_widget else None

        view = self.query_one("#parameter-info", Static)

        if not self.selected_parameter:
            view.update("[red]No parameter is selected yet.[/]")
            return

        if self.selected_parameter == self.current_parameter:
            return

        self.current_parameter = self.selected_parameter
        info_lines = await asyncio.to_thread(self.show_param_info)
        if info_lines:
            view.update(Text.from_markup("\n".join(info_lines)))

    def show_param_info(self) -> List[str]:
        """Retrieve and format parameter information.
        
        Returns:
            List[str]: A list of formatted strings containing parameter information
        """
        
        match = re.fullmatch(r"([^:]+):\s*(.+)", self.current_parameter)
        if not match:
            return [f"[red]Invalid parameter format: {escape(self.current_parameter)}[/]"]
        
        node_name = match.group(1).strip()
        param_name = match.group(2).strip()

        if node_name not in self.param_client_dict:
            client = self.ros_node.create_client(DescribeParameters, f"{node_name}/describe_parameters", callback_group=ReentrantCallbackGroup())
            self.param_client_dict[node_name] = client
        else:
            client = self.param_client_dict[node_name]

        req = DescribeParameters.Request()
        req.names = [param_name]
        future = client.call_async(req)
        self.ros_node.executor.spin_until_future_complete(future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            return [f"[red]Failed to get parameter info for: {escape(self.current_parameter)}[/]"]   

        res = future.result().descriptors[0]
        if res.type == ParameterType.PARAMETER_NOT_SET:
            return [f"[red]Parameter {escape(param_name)} is not set.[/]"]

        field = PARAMETER_TYPE_MAP.get(res.type, None)
        if field is None:
            return [f"[red]Unsupported parameter type for {escape(param_name)}[/]"]
        description = res.description if res.description else "No description available."        

        info_lines = []
        info_lines.append(f"[bold cyan]Parameter name:[/] [yellow]{escape(param_name)}[/]")
        info_lines.append(f"[bold cyan]Type:[/] [green]{escape(field)}[/]")
        info_lines.append(f"[bold cyan]Description:[/] [magenta]{escape(description)}[/]")

        return info_lines
