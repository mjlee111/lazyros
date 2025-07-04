import threading

import rclpy
from rclpy.node import Node
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal, Vertical, ScrollableContainer
from textual.widgets import (
    Footer,
    Header,
    Static,
    TabbedContent,
)

from lazyros.widgets.node_list_widget import NodeListWidget
from lazyros.widgets.log_view_widget import LogViewWidget
from lazyros.widgets.info_view_widget import InfoViewWidget
from lazyros.widgets.topic_list_widget import TopicListWidget
from lazyros.widgets.parameter_list_widget import ParameterListWidget
from lazyros.modals.topic_info_modal import TopicInfoModal  # Import TopicInfoModal
from lazyros.modals.message_modal import MessageModal  # Import MessageModal
from lazyros.utils.utility import ros_spin_thread, signal_shutdown, load_restart_config


class LazyRosApp(App):
    """A Textual app to monitor ROS information."""

    BINDINGS = [
        ("ctrl+q", "quit", "Quit"),
        ("tab", "focus_next_pane", "Next Pane"),
        ("shift+tab", "focus_previous_pane", "Previous Pane"),
    ]

    CSS_PATH = "lazyros.css"

    def __init__(self, ros_node: Node, restart_config=None):
        super().__init__()
        self.ros_node = ros_node
        self.restart_config = load_restart_config("config/restart_config.yaml")
        # List of left pane widgets for tab navigation
        self.left_pane_widgets = [
            "#node-list-content",
            "#topic-list-content", 
            "#parameter-list-content"
        ]
        self.current_pane_index = 0

    def on_mount(self) -> None:
        """Called when app is mounted. Perform async setup here."""
        # Set initial focus to the first (Nodes) list
        node_list_widget = self.query_one("#node-list-content")
        if node_list_widget:
            node_list_widget.node_list_view.focus()

    def on_key(self, event) -> None:
        """Handle key events, override default tab behavior."""
        if event.key == "tab":
            self.action_focus_next_pane()
            event.stop()
        elif event.key == "shift+tab":
            self.action_focus_previous_pane()
            event.stop()

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        yield Header()

        with Horizontal():
            with Container(id="left-frame", classes="left-pane"):
                with Vertical():
                    with Container(classes="list-container"):
                        yield Static("Nodes", classes="frame-title")
                        yield NodeListWidget(self.ros_node, self.restart_config, id="node-list-content")
                    with ScrollableContainer(classes="list-container"):
                        yield Static("Topics", classes="frame-title")
                        yield TopicListWidget(self.ros_node, id="topic-list-content")
                    with Container(classes="list-container"):
                        yield Static("Parameters", classes="frame-title")
                        yield ParameterListWidget(self.ros_node, id="parameter-list-content")

            with Container(id="right-frame", classes="right-pane"):
                yield Static("Logs and Info", classes="frame-title")
                with TabbedContent("Log", "Info"):
                    yield LogViewWidget(self.ros_node, id="log-view-content")
                    yield InfoViewWidget(self.ros_node, id="info-view-content")

        yield Footer()

    def action_toggle_dark(self) -> None:
        """An action to toggle dark mode."""
        self.dark = not self.dark

    def action_restart_node(self) -> None:
        """Forward restart_node action to the NodeListWidget."""
        print("LazyRosApp.action_restart_node: Forwarding action to NodeListWidget")
        node_list = self.query_one(NodeListWidget)
        if node_list:
            node_list.action_restart_node()

    def action_focus_left_pane(self) -> None:
        """Focus the left pane and highlight it."""
        left_pane: Container = self.query_one("#left-frame")
        right_pane: Container = self.query_one("#right-frame")

        left_pane.styles.border = ("heavy", "white")
        right_pane.styles.border = ("solid", "white")

        left_pane.focus()

    def action_focus_right_pane(self) -> None:
        """Focus the right pane and highlight it."""
        left_pane: Container = self.query_one("#left-frame")
        right_pane: Container = self.query_one("#right-frame")

        left_pane.styles.border = ("solid", "white")
        right_pane.styles.border = ("heavy", "white")

        right_pane.focus()

    def action_handle_topic_click(self, topic_name: str) -> None:
        """Handle clicks on topic 'links' in the InfoViewWidget."""
        print(f"Topic clicked: {topic_name}")
        self.push_screen(TopicInfoModal(topic_name))

    def action_handle_message_click(self, message_type: str) -> None:
        """Handle clicks on message type 'links' in the InfoViewWidget."""
        print(f"Message type clicked: {message_type}")
        self.push_screen(MessageModal(message_type))

    def action_focus_next_pane(self) -> None:
        """Focus the next pane in the left panel (Node -> Topics -> Parameters -> Node)."""
        self.current_pane_index = (self.current_pane_index + 1) % len(self.left_pane_widgets)
        self._focus_current_pane()

    def action_focus_previous_pane(self) -> None:
        """Focus the previous pane in the left panel (Parameters -> Topics -> Node -> Parameters)."""
        self.current_pane_index = (self.current_pane_index - 1) % len(self.left_pane_widgets)
        self._focus_current_pane()

    def _focus_current_pane(self) -> None:
        """Focus the current pane based on current_pane_index."""
        try:
            widget_id = self.left_pane_widgets[self.current_pane_index]
            widget = self.query_one(widget_id)
            
            if widget_id == "#node-list-content":
                widget.node_list_view.focus()
            elif widget_id == "#topic-list-content":
                widget.topic_list_view.focus()
            elif widget_id == "#parameter-list-content":
                widget.parameter_list_view.focus()
        except Exception as e:
            print(f"Error focusing pane: {e}")

    def on_descendant_focus(self, event) -> None:
        """Update current pane index when a descendant widget receives focus."""
        try:
            # Check which pane received focus and update the index
            focused_widget = event.widget
            
            # Check if the focused widget is one of our ListViews
            node_widget = self.query_one("#node-list-content")
            topic_widget = self.query_one("#topic-list-content")
            parameter_widget = self.query_one("#parameter-list-content")
            
            if focused_widget == node_widget.node_list_view:
                self.current_pane_index = 0
            elif focused_widget == topic_widget.topic_list_view:
                self.current_pane_index = 1
            elif focused_widget == parameter_widget.parameter_list_view:
                self.current_pane_index = 2
        except Exception:
            pass  # Ignore errors in focus tracking

    # Removed custom run_async method


def main(args=None):
    rclpy.init(args=args)
    ros_node = None
    app: LazyRosApp | None = None  # type: ignore
    ros_thread = None
    try:
        ros_node = Node("lazyros_monitor_node")

        ros_thread = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
        ros_thread.start()

        app = LazyRosApp(ros_node)
        # Run the app using its own run method, which should handle async setup
        app.run()

    except Exception as e:
        print(f"Error initializing ROS or running the TUI: {e}")
    finally:
        signal_shutdown()

        if ros_thread:
            ros_thread.join(timeout=1.0)

        if ros_node:
            ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        print("LazyRos exited cleanly.")


if __name__ == "__main__":
    main()
