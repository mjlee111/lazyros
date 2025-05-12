from rclpy.node import Node
from textual.app import ComposeResult
from textual.containers import Container, VerticalScroll
from textual.widgets import (
    Label,
    ListItem,
    ListView,
)
from rich.markup import escape

def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

class TopicListWidget(Container):
    """A widget to display the list of ROS topics."""

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.topic_list_view = ListView()
        self.previous_topic_names = set() # Store only names

    def compose(self) -> ComposeResult:
        yield Label("ROS Topics:")
        yield VerticalScroll(self.topic_list_view)

    def on_mount(self) -> None:
        self.set_interval(1, self.update_list)
        self.topic_list_view.focus() # Ensure ListView has focus

    def update_list(self) -> None:
        try:
            topic_names_and_types_list = self.ros_node.get_topic_names_and_types()
            
            # We only need names for display and comparison now
            current_topic_names = {name for name, _ in topic_names_and_types_list}

            if current_topic_names != self.previous_topic_names:
                current_index = self.topic_list_view.index
                self.topic_list_view.clear()

                items = []
                if not current_topic_names:
                    items.append(ListItem(Label("[No topics found]")))
                else:
                    for name in sorted(list(current_topic_names)):
                        display_text = escape_markup(name) # Show only topic name
                        items.append(ListItem(Label(display_text, shrink=False)))

                self.topic_list_view.extend(items)

                if current_index is not None and current_index < len(items):
                    self.topic_list_view.index = current_index
                elif len(items) > 0:
                    self.topic_list_view.index = 0

                self.previous_topic_names = current_topic_names

        except Exception as e:
            error_message = f"Error fetching topics: {e}"
            # Avoid flooding with the same error if it persists
            # Check against a simple error flag in previous_topic_names if it's just a set of names
            # For simplicity, let's just check if the set itself is an error message placeholder
            is_previous_error = isinstance(self.previous_topic_names, dict) and self.previous_topic_names.get("error")
            if not is_previous_error or (is_previous_error and self.previous_topic_names.get("error") != error_message) :
                self.topic_list_view.clear()
                self.topic_list_view.append(ListItem(Label(error_message)))
                # Store error differently if previous_topic_names is now a set
                # For simplicity, we can revert to a dict for error state or just display error once.
                # Let's assume if an error occurs, previous_topic_names becomes a marker.
                self.previous_topic_names = {"error": error_message} # Use a dict to store error
