import subprocess
import re 
import asyncio
from typing import List, Optional, Tuple
from concurrent.futures import ThreadPoolExecutor

from rclpy.node import Node
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container, ScrollableContainer
from textual.css.query import DOMQuery
from textual.widgets import (
    Label,
    ListItem,
    ListView,
)
from rich.markup import escape

from lazyros.modals.parameter_value_modal import ParameterValueModal
from lazyros.modals.set_parameter_modal import SetParameterModal
from lazyros.utils.ignore_parser import IgnoreParser
from rcl_interfaces.srv import ListParameters

from dataclasses import dataclass
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy
from rich.text import Text as RichText

def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

@dataclass
class ParameterClients:
    get_parameter: None
    set_parameter: None

class ParameterListWidget(Container):
    """A widget to display the list of ROS parameters."""

    BINDINGS = [
        Binding("s", "set_selected_parameter", "Set Value", show=True),  # Add set binding
    ]

    DEFAULT_CSS = """
    ParameterListWidget {
        overflow: hidden;
    }

    #scroll-area {
        overflow-x: auto;
        overflow-y: auto;
        height: 1fr;
    }
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.listview = ListView()
        self.node_listview = None

        self.parameter_dict = {}
        self.parameter_clients_dict = {}


        self.previous_parameters_display_list: List[str] = []
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="param_list")
        self._current_update_task: Optional[asyncio.Task] = None
        self.ignore_parser = IgnoreParser('config/display_ignore.yaml')
        self.selected_param = None
        self._last_processed_parameter = None
        
        self._highlight_task = None

    def compose(self) -> ComposeResult:
        yield self.listview

    def on_mount(self) -> None:
        #asyncio.create_task(self.update_parameter_list())
        self.set_interval(3, lambda: asyncio.create_task(self.update_parameter_list()))

        self.listview.focus()
        if self.listview.children:
            self.listview.index = 0

    def list_parameters(self, node_name):
        list_parameter_client = self.ros_node.create_client(ListParameters, f"{node_name}/list_parameters", callback_group=ReentrantCallbackGroup())
        
        req = ListParameters.Request()
        future = list_parameter_client.call_async(req)
        rclpy.spin_until_future_complete(self.ros_node, future)
        if not future.done() or future.result() is None:
            return None
        
        result = future.result().result
        return result.names

    async def update_parameter_list(self):
        if not self.node_listview:
            self.node_listview = self.app.query_one("#node-listview")
        
        current_index = self.listview.index
        need_update = False 
        node_list = list(self.node_listview.node_listview_dict.keys())
        for node in node_list:
            if node not in self.parameter_dict:
                need_update = True
                parameters = self.list_parameters(node)
                if parameters:
                    self.parameter_dict[node] = parameters

            elif self.node_listview.node_listview_dict[node].status != "green":
                self.parameter_dict.pop(node)
                need_update = True            

        if not need_update:
            return
        # update parameter listview
        self.listview.clear()
        parameter_list = []
        node_list = list(self.parameter_dict.keys())
        for node in node_list:
            for parameter in self.parameter_dict[node]:
                if not self.ignore_parser.should_ignore(parameter, 'parameter'):
                    label = RichText.assemble(
                        RichText(node),
                        ": ",
                        RichText(parameter)
                    )
                    parameter_list.append(ListItem(Label(label)))

        self.listview.extend(parameter_list)
        self.listview.index = 0

    def _parse_selected_item(self, item_text: str) -> Optional[Tuple[str, str]]:
        """Regex to capture node_name and param_name from "node_name: param_name."""

        match = re.fullmatch(r"([^:]+):\s*(.+)", item_text)
        if match:
            node_name = match.group(1).strip()
            param_name = match.group(2).strip()
            return node_name, param_name
        return None

    def on_list_view_highlighted(self, event):
        index = self.listview.index
        if index is None or not (0 <= index < len(self.listview.children)):
            self.selected_parame = None
            return
        item = self.listview.children[index]
        if not item.children:
            self.selected_param = None
            return

        param_name = str(item.children[0].renderable).strip()

        if self.selected_param != param_name:
            self.selected_param = param_name
        self.update_window_display()


    def update_window_display(self):
        """Update main window display"""

        if not self.selected_param or not ":" in self.selected_param:
            self.log.error("No valid parameter selected for display update.")
            return

        try:
            if self.app.current_right_pane_config == "parameter":
                try:
                    info_widget = self.app.query_one("#parameter-info-view-content")
                    info_widget.current_parameter = self.selected_param
                
                    value_widget = self.app.query_one("#parameter-value-view-content")
                    value_widget.current_parameter = self.selected_param

                except Exception:
                    self.log.error("Error updating parameter display widgets.")

        except Exception:
            self.log.error("Error updating parameter display in main window.")


    # --- Action to Set Parameter Value ---
    def action_set_selected_parameter(self) -> None:
        """Action to set the value of the currently selected parameter."""

        highlighted_item_widget: Optional[ListItem] = self.listview.highlighted_child
        if not highlighted_item_widget:
            self.app.bell()
            return

        children_query: DOMQuery[Label] = highlighted_item_widget.query(Label)  # type: ignore
        if not children_query:
            self.app.bell()
            return

        selected_label: Label = children_query.first()
        item_text_renderable = selected_label.renderable
        item_text_plain = str(item_text_renderable)  # Convert RichText or str to plain str

        parsed_names = self._parse_selected_item(item_text_plain)
        if not parsed_names:
            self.app.bell()
            self.app.push_screen(ParameterValueModal(title="Error", content="Could not parse selected parameter string."))
            return

        node_name, param_name = parsed_names

        try:
            cmd = f"ros2 param describe \"{node_name}\" \"{param_name}\""
            process_result = subprocess.run(
                cmd, shell=True, capture_output=True, text=True, timeout=5
            )

            if process_result.returncode == 0:
                # Parse the output to get the type
                # Example output:
                # Parameter: use_sim_time
                #   Type: bool
                #   Description: Use simulation time
                description_output = process_result.stdout.strip()
                type_match = re.search(r"Type: (\w+)", description_output)
                param_type = type_match.group(1) if type_match else "Unknown"

                self.app.push_screen(SetParameterModal(node_name, param_name, param_type))

            else:
                err_msg = process_result.stderr.strip() if process_result.stderr else "Unknown error"
                # self._log_error(f"Error describing param {node_name} {param_name}: RC {process_result.returncode}, Err: {err_msg}")
                self.app.push_screen(ParameterValueModal(title="Error", content=f"Could not describe parameter:\n{err_msg}"))

        except subprocess.TimeoutExpired:
            self.app.push_screen(ParameterValueModal(title="Error", content="Timeout describing parameter."))
        except Exception as e:
            self.app.push_screen(ParameterValueModal(title="Error", content=f"An error occurred describing parameter: {e}"))