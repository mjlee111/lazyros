import subprocess
import threading
import re # For parsing node and param name
from typing import List, Optional, Tuple

from rclpy.node import Node
from textual.app import ComposeResult, App
from textual.binding import Binding
from textual.containers import Container, VerticalScroll, Horizontal, Center, ScrollableContainer
from textual.css.query import DOMQuery
from textual.reactive import reactive
from textual.screen import Screen
from modals.parameter_value_modal import ParameterValueModal
from modals.set_parameter_modal import SetParameterModal # Import SetParameterModal
from utils.ignore_parser import IgnoreParser # Import IgnoreParser
from textual.widgets import (
    Label,
    ListItem,
    ListView,
    Static,
    Button,
    # Removed Input as it's not used here
)
from rich.markup import escape

def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

# --- Modal Screen Definition ---
class ParameterValueModalScreen(Screen):
    """A modal screen to display a parameter's value."""

    BINDINGS = [
        Binding("escape,q", "pop_screen", "Close", show=True),
    ]

    def __init__(self, title: str, content: str, **kwargs) -> None:
        super().__init__(**kwargs)
        self.modal_title = title
        self.modal_content = content

    def compose(self) -> ComposeResult:
        yield Container(
            Static(self.modal_title, id="modal_title"),
            Container(
                Static(self.modal_content, id="modal_content_text"), # Wrap content for scrolling if needed
                id="modal_content_container"
            ),
            Center(
                Button("Close", variant="primary", id="modal_close_button")
            ),
            id="modal_dialog",
        )

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "modal_close_button":
            self.app.pop_screen()

# --- ParameterListWidget ---
class ParameterListWidget(Container):
    """A widget to display the list of ROS parameters using 'ros2 param list'."""

    BINDINGS = [
        Binding("g", "get_selected_parameter_value", "Get Value", show=True),
        Binding("s", "set_selected_parameter", "Set Value", show=True), # Add set binding
    ]

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.parameter_list_view = ListView()
        self.previous_parameters_display_list: List[str] = []
        self._update_thread: Optional[threading.Thread] = None
        self._is_updating_lock = threading.Lock()
        self._is_updating = False
        self._get_value_thread: Optional[threading.Thread] = None
        self.ignore_parser = IgnoreParser('config/display_ignore.yaml') # Instantiate IgnoreParser

    def _log_error(self, msg: str):
        if hasattr(self.ros_node, 'get_logger'):
            self.ros_node.get_logger().error(f"[ParameterListWidget] {msg}")

    def compose(self) -> ComposeResult:
        yield Label("ROS Parameters:")
        yield ScrollableContainer(self.parameter_list_view)
        #self.parameter_list_view.focus() # Ensure list view can receive key presses

    def on_mount(self) -> None:
        self.set_interval(3, self.trigger_update_list)

    def _parse_ros2_param_list_output(self, output: str) -> List[str]:
        parsed_params: List[str] = []
        current_node_name = None
        lines = output.splitlines()

        for i, line_raw in enumerate(lines):
            line = line_raw.strip()
            if not line:
                continue

            if line.endswith(':'):
                potential_node_name = line[:-1].strip()
                if potential_node_name.startswith('/'):
                    current_node_name = potential_node_name
                else:
                    current_node_name = None

            elif current_node_name and line:
                if line_raw.startswith("  ") and not line.startswith(" "):
                    param_name = line.strip()
                    if param_name:
                        # Store raw text for parsing later, display escaped text
                        parsed_params.append(escape_markup(f"{current_node_name}: {param_name}"))

        parsed_params.sort()
        return parsed_params

    def _fetch_and_parse_parameters(self) -> List[str]:
        try:
            cmd = "ros2 param list"
            process_result = subprocess.run(
                cmd, shell=True, capture_output=True, text=True, timeout=10
            )
            #if process_result.stderr:
                #self._log_error(f"Thread: stderr from 'ros2 param list':\n{process_result.stderr}")

            if process_result.returncode == 0:
                if process_result.stdout:
                    output_str = process_result.stdout.strip()
                    if output_str:
                        parsed_list = self._parse_ros2_param_list_output(output_str)
                        
                        # Filter parameters based on the ignore list
                        filtered_params = [
                            param_str for param_str in parsed_list
                            if not self.ignore_parser.should_ignore(param_str, 'parameter')
                        ]

                        if not filtered_params:
                            return ["[No parameters found after filtering]"]
                        return filtered_params
                    else:
                        return ["[No parameters found: 'ros2 param list' returned empty output]"]
                else:
                    return ["['ros2 param list' succeeded but gave no stdout]"]
            else:
                err_msg_raw = process_result.stderr.strip() if process_result.stderr else "Unknown error"
                #self._log_error(f"Thread: 'ros2 param list' failed. RC: {process_result.returncode}. Error: {err_msg_raw}")
                return [escape_markup(f"[Error (RC {process_result.returncode}) running 'ros2 param list'. See logs]")]

        except subprocess.TimeoutExpired:
            #self._log_error("Thread: 'ros2 param list' command timed out.")
            return ["[Error: 'ros2 param list' command timed out. Check ROS environment]"]

        except Exception as e_thread:
            #self._log_error(f"Thread: Error during parameter list fetch: {type(e_thread).__name__} - {str(e_thread)}")
            return [escape_markup(f"[General Error in list fetch thread: {type(e_thread).__name__}. See logs]")]

    def _update_view_from_thread(self, new_params_list: List[str]):
        with self._is_updating_lock:
            self._is_updating = False

        if self.previous_parameters_display_list != new_params_list:
            self.parameter_list_view.clear()
            items = [ListItem(Label(param_str)) for param_str in new_params_list] if new_params_list else [ListItem(Label("[No parameters available or error during fetch]"))]
            self.parameter_list_view.extend(items)
            if items and (self.parameter_list_view.index is None or self.parameter_list_view.index >= len(items)):
                self.parameter_list_view.index = 0
            elif not items:
                 self.parameter_list_view.index = None
            self.previous_parameters_display_list = new_params_list

    def _list_thread_target(self):
        result = self._fetch_and_parse_parameters()
        if self.app:
            self.app.call_from_thread(self._update_view_from_thread, result)

    def trigger_update_list(self) -> None:
        with self._is_updating_lock:
            if self._is_updating: return
            self._is_updating = True

        self._update_thread = threading.Thread(target=self._list_thread_target, daemon=True)
        self._update_thread.start()

    # --- Get Parameter Value Functionality ---
    def _parse_selected_item(self, item_text: str) -> Optional[Tuple[str, str]]:
        """Regex to capture node_name and param_name from "node_name: param_name."""

        match = re.fullmatch(r"([^:]+):\s*(.+)", item_text)
        if match:
            node_name = match.group(1).strip()
            param_name = match.group(2).strip()
            return node_name, param_name
        #self._log_error(f"Could not parse selected item: '{item_text}'")
        return None

    def _fetch_parameter_value_thread_target(self, node_name: str, param_name: str, modal_instance: ParameterValueModal):
        """Fetches a single parameter value in a thread and updates the modal."""

        value_result_str = f"Fetching value for {node_name}: {param_name}..."
        try:
            cmd = f"ros2 param get \"{node_name}\" \"{param_name}\"" # Ensure quoting for names with spaces/symbols
            process_result = subprocess.run(
                cmd, shell=True, capture_output=True, text=True, timeout=5
            )

            title = f"Value for {param_name}"
            if process_result.returncode == 0:
                value_result_str = process_result.stdout.strip() if process_result.stdout else "[No output from command]"
            else:
                err_msg = process_result.stderr.strip() if process_result.stderr else "Unknown error"
                #self._log_error(f"Error getting param {node_name} {param_name}: RC {process_result.returncode}, Err: {err_msg}")
                value_result_str = f"Error fetching value:\n{err_msg}"
        except subprocess.TimeoutExpired:
            #self._log_error(f"Timeout getting param {node_name} {param_name}")
            title = f"Timeout for {param_name}"
            value_result_str = "Command timed out."
        except Exception as e:
            #self._log_error(f"Exception getting param {node_name} {param_name}: {e}")
            title = f"Exception for {param_name}"
            value_result_str = f"An error occurred: {e}"

        # Update the content of the existing modal
        if self.app:
            self.app.call_from_thread(modal_instance.update_content, title, value_result_str)

    # Removed _show_parameter_value_modal as it's no longer needed

    def action_get_selected_parameter_value(self) -> None:
        """Action to get the value of the currently selected parameter."""

        highlighted_item_widget: Optional[ListItem] = self.parameter_list_view.highlighted_child
        if not highlighted_item_widget:
            self.app.bell()
            return

        # ListItem contains a Label. We need the Label's text.
        children_query: DOMQuery[Label] = highlighted_item_widget.query(Label) # type: ignore
        if not children_query:
            self.app.bell()
            return

        selected_label: Label = children_query.first()
        item_text_renderable = selected_label.renderable
        item_text_plain = str(item_text_renderable) # Convert RichText or str to plain str

        parsed_names = self._parse_selected_item(item_text_plain)
        if not parsed_names:
            self.app.bell()
            self.app.push_screen(ParameterValueModal(title="Error", content="Could not parse selected parameter string."))
            return

        node_name, param_name = parsed_names

        fetching_modal = ParameterValueModal(title=f"Fetching {param_name}...", content="Please wait...")
        self.app.push_screen(fetching_modal)

        self._get_value_thread = threading.Thread(
            target=self._fetch_parameter_value_thread_target,
            args=(node_name, param_name, fetching_modal), # Pass modal instance to thread
            daemon=True
        )
        self._get_value_thread.start()

    def action_set_selected_parameter(self) -> None:
        """Action to set the value of the currently selected parameter."""
        highlighted_item_widget: Optional[ListItem] = self.parameter_list_view.highlighted_child
        if not highlighted_item_widget:
            self.app.bell()
            return

        children_query: DOMQuery[Label] = highlighted_item_widget.query(Label) # type: ignore
        if not children_query:
            self.app.bell()
            return

        selected_label: Label = children_query.first()
        item_text_renderable = selected_label.renderable
        item_text_plain = str(item_text_renderable) # Convert RichText or str to plain str

        parsed_names = self._parse_selected_item(item_text_plain)
        if not parsed_names:
            self.app.bell()
            self.app.push_screen(ParameterValueModal(title="Error", content="Could not parse selected parameter string."))
            return

        node_name, param_name = parsed_names

        # Fetch parameter type using ros2 param describe
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

                # Push the set parameter modal
                self.app.push_screen(SetParameterModal(node_name, param_name, param_type))

            else:
                err_msg = process_result.stderr.strip() if process_result.stderr else "Unknown error"
                #self._log_error(f"Error describing param {node_name} {param_name}: RC {process_result.returncode}, Err: {err_msg}")
                self.app.push_screen(ParameterValueModal(title="Error", content=f"Could not describe parameter:\n{err_msg}"))

        except subprocess.TimeoutExpired:
            #self._log_error(f"Timeout describing param {node_name} {param_name}")
            self.app.push_screen(ParameterValueModal(title="Error", content="Timeout describing parameter."))
        except Exception as e:
            #self._log_error(f"Exception describing param {node_name} {param_name}: {e}")
            self.app.push_screen(ParameterValueModal(title="Error", content=f"An error occurred describing parameter: {e}"))


    def on_unmount(self) -> None:
        pass
