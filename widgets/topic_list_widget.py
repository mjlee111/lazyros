from rclpy.node import Node
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container, VerticalScroll
from textual.widgets import (
    Label,
    ListItem,
    ListView,
)
from rich.markup import escape
import subprocess
from modals import TopicInfoModal, TopicEchoModal # Import the modals

def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

class TopicListWidget(Container):
    """A widget to display the list of ROS topics."""

    BINDINGS = [
        Binding("i", "show_topic_info", "Info"),
        Binding("e", "echo_topic", "Echo"),
    ]

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.topic_list_view = ListView()
        self.previous_topic_data: dict[str, str] = {} # Stores {topic_name: first_type_str} or {"error": msg}

    def compose(self) -> ComposeResult:
        yield Label("ROS Topics:")
        yield VerticalScroll(self.topic_list_view)

    def on_mount(self) -> None:
        self.set_interval(1, self.update_list)
        self.topic_list_view.focus() # Ensure ListView has focus

    def update_list(self) -> None:
        try:
            topic_names_and_types_list = self.ros_node.get_topic_names_and_types()
            
            current_topic_data: dict[str, str] = {}
            for name, types_list in topic_names_and_types_list:
                if types_list: # Ensure there's at least one type
                    current_topic_data[name] = types_list[0] 
                else:
                    current_topic_data[name] = "" # Placeholder if no types (should be rare)

            if current_topic_data != self.previous_topic_data:
                current_index = self.topic_list_view.index
                self.topic_list_view.clear()

                items = []
                if not current_topic_data:
                    items.append(ListItem(Label("[No topics found]")))
                else:
                    # Display only names, sorted
                    for name in sorted(current_topic_data.keys()):
                        display_text = escape_markup(name) 
                        items.append(ListItem(Label(display_text, shrink=False)))

                self.topic_list_view.extend(items)

                if current_index is not None and current_index < len(items):
                    self.topic_list_view.index = current_index
                elif len(items) > 0:
                    self.topic_list_view.index = 0
                
                self.previous_topic_data = current_topic_data

        except Exception as e:
            error_message = f"Error fetching topics: {e}"
            if self.previous_topic_data.get("error") != error_message:
                self.topic_list_view.clear()
                self.topic_list_view.append(ListItem(Label(error_message)))
                self.previous_topic_data = {"error": error_message}

    def action_show_topic_info(self) -> None:
        """Show detailed information about the selected topic in a modal."""
        if self.topic_list_view.index is None:
            return
        
        # Check if previous_topic_data contains an error
        if self.previous_topic_data.get("error") is not None:
            self.app.bell()
            return

        # Ensure previous_topic_data is not empty and is a dict of topics
        if not self.previous_topic_data or "error" in self.previous_topic_data:
             self.app.bell()
             return

        sorted_names = sorted(list(self.previous_topic_data.keys()))
        if not (0 <= self.topic_list_view.index < len(sorted_names)):
            return

        selected_topic_name = sorted_names[self.topic_list_view.index]
        if selected_topic_name == "[No topics found]" or not selected_topic_name.startswith("/"):
            # Add a message to the app's log or a status bar if available
            self.app.bell() # Simple feedback
            return
        
        self.app.push_screen(TopicInfoModal(topic_name=selected_topic_name))

    def action_echo_topic(self) -> None:
        """Echo the selected topic in a modal dialog."""
        if self.topic_list_view.index is None:
            return

        if self.previous_topic_data.get("error") is not None:
            self.app.bell()
            return
        
        if not self.previous_topic_data or "error" in self.previous_topic_data:
            self.app.bell()
            return

        sorted_names = sorted(list(self.previous_topic_data.keys()))
        if not (0 <= self.topic_list_view.index < len(sorted_names)):
            return
            
        selected_topic_name = sorted_names[self.topic_list_view.index]
        if selected_topic_name == "[No topics found]" or not selected_topic_name.startswith("/"):
            self.app.bell()
            return

        selected_topic_type = self.previous_topic_data.get(selected_topic_name)
        if selected_topic_type is None: # Check for None explicitly
            # self.app.notify(f"Error: No type found for topic {selected_topic_name}", severity="error", timeout=3)
            self.app.bell()
            return
        
        # Even if type is empty string, pass it to modal. Modal will handle it.
        self.app.push_screen(TopicEchoModal(topic_name=selected_topic_name, topic_type=selected_topic_type))
