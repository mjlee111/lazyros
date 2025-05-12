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

class ParameterListWidget(Container):
    """A widget to display the list of ROS parameters.
    Note: This is a simplified version. Listing all parameters for all nodes
    can be complex. This might be enhanced later to show parameters for a selected node.
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node # The TUI's own node, not for listing its params
        self.parameter_list_view = ListView()
        self.previous_parameter_data = {} # Could store more complex data if needed

    def compose(self) -> ComposeResult:
        yield Label("ROS Parameters:")
        yield VerticalScroll(self.parameter_list_view)

    def on_mount(self) -> None:
        self.set_interval(1, self.update_list) # Update less frequently, can be intensive
        self.parameter_list_view.focus() # Or remove if focus should be managed elsewhere

    def update_list(self) -> None:
        """Fetches and updates the list of ROS parameters.
        Currently, this is a placeholder as listing all parameters for all nodes
        is not straightforward. It might show parameters for a selected node in the future.
        """
        try:
            # Placeholder implementation:
            # A full implementation would iterate self.app.query_one(NodeListWidget).previous_node_names,
            # then for each node, use self.ros_node.list_parameters(node_name)
            # and self.ros_node.get_parameters(node_name, param_names)
            # This is complex and can be slow.
            
            # For now, display a placeholder message or parameters of the TUI node itself (less useful).
            # Let's stick to a placeholder to indicate it's not fully implemented.
            
            placeholder_message = "[Parameter listing per node not yet implemented]"
            
            # Only update if the message changes (e.g., if we implement actual listing later)
            if self.previous_parameter_data.get("message") != placeholder_message:
                self.parameter_list_view.clear()
                items = [ListItem(Label(placeholder_message))]
                self.parameter_list_view.extend(items)
                if len(items) > 0: # Should always be true here
                    self.parameter_list_view.index = 0
                self.previous_parameter_data = {"message": placeholder_message}

        except Exception as e:
            error_message = f"Error in parameter widget: {e}"
            if self.previous_parameter_data.get("error") != error_message:
                self.parameter_list_view.clear()
                self.parameter_list_view.append(ListItem(Label(error_message)))
                self.previous_parameter_data = {"error": error_message}
