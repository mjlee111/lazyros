import subprocess
import re  # For parsing node and param name
import asyncio
from typing import List, Optional, Tuple
from concurrent.futures import ThreadPoolExecutor

from rclpy.node import Node
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Center, Container, ScrollableContainer
from textual.css.query import DOMQuery
from textual.screen import Screen
from textual.widgets import (
    Label,
    ListItem,
    ListView,
    Static,
    Button,
    # Removed Input as it's not used here
)
from rich.markup import escape

from lazyros.modals.parameter_value_modal import ParameterValueModal
from lazyros.modals.set_parameter_modal import SetParameterModal  # Import SetParameterModal
from lazyros.utils.ignore_parser import IgnoreParser  # Import IgnoreParser


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
                Static(self.modal_content, id="modal_content_text"),  # Wrap content for scrolling if needed
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


class ParameterListWidget(Container):
    """A widget to display the list of ROS parameters using 'ros2 param list'."""

    BINDINGS = [
        Binding("s", "set_selected_parameter", "Set Value", show=True),  # Add set binding
    ]

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.parameter_list_view = ListView()
        self.previous_parameters_display_list: List[str] = []
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="param_list")
        self._current_update_task: Optional[asyncio.Task] = None
        self.ignore_parser = IgnoreParser('config/display_ignore.yaml')
        self.selected_parameter_text = None
        self._current_parameter = None
        
        self._highlight_task = None
        self._highlight_delay = 1  # 0.5 second delay for better responsiveness

    def _log_error(self, msg: str):
        if hasattr(self.ros_node, 'get_logger'):
            self.ros_node.get_logger().error(f"[ParameterListWidget] {msg}")

    def compose(self) -> ComposeResult:
        yield Label("ROS Parameters:")
        yield ScrollableContainer(self.parameter_list_view)

    def on_mount(self) -> None:
        self.parameter_list_view.initial_index = 0
        self.set_interval(5, self.trigger_update_list)
        # Trigger initial update and ensure first item is highlighted
        self.trigger_update_list()

    def _parse_ros2_param_list_output(self, output: str) -> List[str]:
        parsed_params: List[str] = []
        current_node_name = None
        lines = output.splitlines()

        for line_raw in lines:
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
                err_msg = process_result.stderr.strip() if process_result.stderr else "Unknown error"
                # self._log_error(f"Thread: 'ros2 param list' failed. RC: {process_result.returncode}. Error: {err_msg}")
                return [escape_markup(f"[Error (RC {process_result.returncode}) running 'ros2 param list'. See logs]")]

        except subprocess.TimeoutExpired:
            # self._log_error("Thread: 'ros2 param list' command timed out.")
            return ["[Error: 'ros2 param list' command timed out. Check ROS environment]"]

        except Exception as e:
            # self._log_error(f"Thread: Error during parameter list fetch: {type(e).__name__} - {str(e)}")
            return [escape_markup(f"[General Error in list fetch thread: {type(e).__name__}. See logs]")]

    def _update_view(self, new_params_list: List[str]):
        """Update the parameter list view with new data."""
        if self.previous_parameters_display_list != new_params_list:
            self.parameter_list_view.clear()
            items = [ListItem(Label(param_str)) for param_str in new_params_list] if new_params_list else [ListItem(Label("[No parameters available or error during fetch]"))]
            self.parameter_list_view.extend(items)
            if items and (self.parameter_list_view.index is None or self.parameter_list_view.index >= len(items)):
                self.parameter_list_view.index = 0
            elif not items:
                self.parameter_list_view.index = None
            self.previous_parameters_display_list = new_params_list

    async def _update_list_async(self) -> None:
        """Asynchronously update the parameter list."""
        try:
            # Run the fetch operation in a thread pool
            result = await asyncio.get_event_loop().run_in_executor(
                self._executor, self._fetch_and_parse_parameters
            )
            # Update the UI in the main thread
            self._update_view(result)
        except asyncio.CancelledError:
            # Task was cancelled, ignore
            pass
        except Exception as e:
            self._log_error(f"Error during async parameter list update: {e}")
            # Show error in UI
            self._update_view([f"[Error during update: {e}]"])

    def trigger_update_list(self) -> None:
        """Trigger an update of the parameter list, cancelling any existing update."""
        # Cancel existing update task if running
        if self._current_update_task and not self._current_update_task.done():
            self._current_update_task.cancel()
        
        # Start new update task
        self._current_update_task = asyncio.create_task(self._update_list_async())

    # --- Get Parameter Value Functionality ---
    def _parse_selected_item(self, item_text: str) -> Optional[Tuple[str, str]]:
        """Regex to capture node_name and param_name from "node_name: param_name."""

        match = re.fullmatch(r"([^:]+):\s*(.+)", item_text)
        if match:
            node_name = match.group(1).strip()
            param_name = match.group(2).strip()
            return node_name, param_name
        # self._log_error(f"Could not parse selected item: '{item_text}'")
        return None

    def action_set_selected_parameter(self) -> None:
        """Action to set the value of the currently selected parameter."""
        highlighted_item_widget: Optional[ListItem] = self.parameter_list_view.highlighted_child
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

    def on_list_view_highlighted(self, event):
        """Handle when a parameter is highlighted/selected in the ListView."""
        try:
            index = self.parameter_list_view.index
            
            if index is None or index < 0 or index >= len(self.parameter_list_view.children):
                self.selected_parameter_text = None
                return
            
            selected_item = self.parameter_list_view.children[index]
            if not selected_item.children:
                self.selected_parameter_text = None
                return
            
            child = selected_item.children[0]
            parameter_text = str(child.renderable).strip()
            
            # Skip if same parameter is selected
            if self._current_parameter == parameter_text:
                return
                
            # Only process if it's a valid parameter (contains ":")
            if ":" in parameter_text and not parameter_text.startswith("["):
                self.selected_parameter_text = parameter_text
                self._current_parameter = parameter_text
                
                # Use delayed update mechanism like node_list_widget and topic_list_widget
                if self._highlight_task and not self._highlight_task.done():
                    self._highlight_task.cancel()
                self._highlight_task = asyncio.create_task(self._delayed_update())
            else:
                self.selected_parameter_text = None
                self._current_parameter = None
                
        except Exception:
            self.selected_parameter_text = None
            self._current_parameter = None

    def on_list_view_selected(self, event):
        """Handle when a parameter is selected in the ListView."""
        self.on_list_view_highlighted(event)

    async def _delayed_update(self):
        await asyncio.sleep(self._highlight_delay)
        await self._update_parameter_display_async()

    async def _update_parameter_display_async(self):
        """Update the parameter display asynchronously."""
        if not self.selected_parameter_text or not ":" in self.selected_parameter_text:
            return

        try:
            if hasattr(self.app, 'current_right_pane_config') and self.app.current_right_pane_config == "parameters":
                try:
                    param_info_widget = self.app.query_one("#parameter-info-view-content")
                    param_info_widget.update_parameter_info(self.selected_parameter_text)
                
                    value_widget = self.app.query_one("#parameter-value-view-content")
                    value_widget.update_parameter(self.selected_parameter_text)
                except Exception:
                    pass
            
            # Fallback to main app update method
            if hasattr(self.app, 'update_parameter_display'):
                self.app.update_parameter_display(self.selected_parameter_text)
        except Exception:
            pass

    def on_unmount(self) -> None:
        """Clean up resources when the widget is unmounted."""
        # Cancel any running tasks
        if self._current_update_task and not self._current_update_task.done():
            self._current_update_task.cancel()
        if self._highlight_task and not self._highlight_task.done():
            self._highlight_task.cancel()
        
        # Shutdown the thread pool
        self._executor.shutdown(wait=False)
