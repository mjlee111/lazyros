import atexit
import os
import signal
from typing import Any

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal, ScrollableContainer, Vertical
from textual.reactive import reactive
from textual.screen import ModalScreen
from textual.widgets import Header, Static, TabbedContent, TabPane

from lazyros.utils.custom_widgets import CustomListView, SearchFooter
from lazyros.utils.utility import RosRunner
from lazyros.widgets.node.node_info import InfoViewWidget
from lazyros.widgets.node.node_lifecycle import LifecycleWidget
from lazyros.widgets.node.node_list import NodeListWidget
from lazyros.widgets.node.node_log import LogViewWidget
from lazyros.widgets.parameter.parameter_info import ParameterInfoWidget
from lazyros.widgets.parameter.parameter_list import ParameterListWidget
from lazyros.widgets.parameter.parameter_value import ParameterValueWidget
from lazyros.widgets.topic.topic_echo import EchoViewWidget
from lazyros.widgets.topic.topic_info import TopicInfoWidget
from lazyros.widgets.topic.topic_list import TopicListWidget


ros_runner = RosRunner()
atexit.register(ros_runner.stop)

class HelpModal(ModalScreen):
    CSS = """
    HelpModal { align: center middle; layer: modal; }
    #modal-container {
        width: auto; height: auto;
        border: round white;
        background: $background;
    }
    """
    BINDINGS = [Binding("escape", "dismiss", "Quit Modal", priority=True)]

    def compose(self) -> ComposeResult:
        """Compose the help modal with keyboard shortcuts information.
        
        Returns:
            ComposeResult: The composed widget structure for the help modal.
        """
        help_text = (
            "Help Menu\n"
            "\n"
            "enter        Focus right window\n"
            "\[            Previous Tab\n"
            "]            Next Tab\n"
            "tab          Focus next container\n"
            "shift+tab    Focus previous container"
        )
        yield Static(help_text, id="modal-container")


class LazyRosApp(App):
    """A Textual app to monitor ROS information."""
    CSS_PATH = "lazyros.css"

    BINDINGS = [
        Binding("ctrl+q", "quit", "Quit", show=True, priority=True),
        Binding("?", "help", "Help", show=True),
        Binding("/", "search", "Search", show=False),
        Binding("tab", "focus_next_listview", "Focus Next ListView", show=False, priority=True),
        Binding("shift+tab", "focus_previous_listview", "Focus Previous ListView", show=False, priority=True),
    ]

    LISTVIEW_CONTAINERS = ["node", "topic", "parameter"]
    TAB_ID_DICT = {
        "node": ["log", "lifecycle", "info"],
        "topic": ["echo", "info"],
        "parameter": ["value", "info"],
    }

    focused_pane = reactive("left")
    current_pane_index = reactive(0)

    def __init__(self, **kwargs: Any) -> None:
        """Initialize the LazyROS application.
        
        Args:
            **kwargs: Keyword arguments passed to the parent App class.
            
        Raises:
            Exception: If ROS node initialization fails.
        """
        super().__init__(**kwargs)

        self._searching = False
        try:
            ros_runner.start()
            self.ros_node = ros_runner.node
            assert self.ros_node is not None, "ROS node must be available before compose()"
        except Exception as e:
            self.exit(message=f"Failed to initialize ROS node: {e}")

    def get_ros_info(self) -> str:
        """Get ROS environment information for display in the title bar.
        
        Returns:
            str: Formatted string containing ROS distribution, domain ID, and DDS implementation.
        """
        ros_distro = os.environ.get("ROS_DISTRO", "unknown")
        ros_domain = os.environ.get("ROS_DOMAIN_ID", "0")
        dds_implementation = os.environ.get("RMW_IMPLEMENTATION", "unknown")
        title = f"ROS_DISTRO={ros_distro}  |  ROS_DOMAIN_ID={ros_domain}  |  DDS_IMPLEMENTATION={dds_implementation}"
        return title

    def on_mount(self) -> None:
        """Handle the mount event by initializing the UI and setting focus.
        
        Sets the window title with ROS information and focuses the first node list widget.
        """
        try:
            self.screen.title = self.get_ros_info()
            self.right_pane = self.query_one("#right-pane")

            node_list_widget = self.query_one("#node-listview")
            node_list_widget.listview.focus()
        except Exception as e:
            self.log.error(f"Error during mount: {e}")

    def on_shutdown(self, _event: Any) -> None:
        """Handle application shutdown by stopping the ROS runner.
        
        Args:
            _event: The shutdown event (unused).
        """
        ros_runner.stop()

    def on_mouse_down(self, event: Any) -> None:
        """Handle mouse down events to manage pane focus.
        
        When clicking on widgets other than list views or search footer,
        automatically focus the right pane.
        
        Args:
            event: The mouse down event.
        """
        focus_type = type(self.screen.focused)

        if focus_type is not CustomListView and focus_type is not SearchFooter:
            self.focused_pane = "right"
        
    def on_key(self, event: Any) -> None:
        """Handle key events for navigation and search functionality.
        
        Provides keyboard shortcuts for tab navigation, search, and pane focusing.
        
        Args:
            event: The key event containing key information.
        """
        if event.key == "enter":
            if self._searching:
                self.focus_searched_listview()
            else:
                self.action_focus_right_pane()
            event.stop()
        elif event.character == "[":
            self.action_previous_tab()
            event.stop()
        elif event.character == "]":
            self.action_next_tab()
            event.stop()
        elif event.key == "escape" and self._searching:
            self.end_search()
            event.stop()

    def compose(self) -> ComposeResult:
        """Compose the main application UI with all widgets and containers.
        
        Creates the layout with left pane containing node/topic/parameter lists
        and right pane with tabbed content for detailed information.
        
        Returns:
            ComposeResult: The composed widget structure.
        """

        yield Header(icon="", id="header")

        with Horizontal():
            with Container(classes="left-pane", id="left-pane"):
                with Vertical():
                    node_container = ScrollableContainer(classes="list-container", id="node-container")
                    node_container.border_title = "Nodes"
                    with node_container:
                        yield NodeListWidget(self.ros_node, classes="list-view", id="node-listview")

                    topic_container = ScrollableContainer(classes="list-container", id="topic-container")
                    topic_container.border_title = "Topics"
                    with topic_container:
                        yield TopicListWidget(self.ros_node, classes="list-view", id="topic-listview")

                    parameter_container = ScrollableContainer(classes="list-container", id="parameter-container")
                    parameter_container.border_title = "Parameters"
                    with parameter_container:
                        yield ParameterListWidget(self.ros_node, classes="list-view", id="parameter-listview")

            with Container(classes="right-pane", id="right-pane"):
                with TabbedContent("Log", "Lifecycle", "Info", id="node-tabs"):
                    with TabPane("Log", id="log"):
                        yield LogViewWidget(self.ros_node, classes="view-content", id="node-log-view-content")
                    with TabPane("Lifecycle", id="lifecycle"):
                        yield LifecycleWidget(self.ros_node, classes="view-content", id="node-lifecycle-view-content")
                    with TabPane("Info", id="info"):
                        yield InfoViewWidget(self.ros_node, classes="view-content", id="node-info-view-content")

                with TabbedContent("Info", "Echo", id="topic-tabs", classes="hidden"):
                    with TabPane("Echo", id="echo"):
                        yield EchoViewWidget(self.ros_node, classes="view-content", id="topic-echo-view-content")
                    with TabPane("Info", id="info"):
                        yield TopicInfoWidget(self.ros_node, classes="view-content", id="topic-info-view-content")

                with TabbedContent("Info", "Value", id="parameter-tabs", classes="hidden"):
                    with TabPane("Value", id="value"):
                        yield ParameterValueWidget(self.ros_node, classes="view-content", id="parameter-value-view-content")
                    with TabPane("Info", id="info"):
                        yield ParameterInfoWidget(self.ros_node, classes="view-content", id="parameter-info-view-content")

        self.footer = SearchFooter(id="footer")
        yield self.footer 

    def action_search(self) -> None:
        """Activate search mode for the currently focused list view.
        
        Enables search functionality and focuses the search footer.
        """
        self._searching = True 

        searching_listview = self.LISTVIEW_CONTAINERS[self.current_pane_index] + "-listview"
        self.footer.searching_id = searching_listview
        listview = self.query_one(f"#{searching_listview}")
        listview.searching = True

        self.footer.enter_search()
        self.set_focus(self.footer)

    def focus_searched_listview(self) -> None:
        """Focus the list view that is currently being searched.
        
        Called when Enter is pressed during search mode to return focus
        to the searched list view.
        """
        try:
            listview_id = self.footer.searching_id
            if listview_id:
                listview = self.query_one(f"#{listview_id}")
                listview.listview.focus()
        except Exception as e:
            self.log.error(f"Error focusing searched listview: {e}")
        
    def end_search(self) -> None:
        """Exit search mode and return to normal navigation.
        
        Clears the search state, exits search mode on the list view,
        and returns focus to the list view.
        """
        self.footer.exit_search()
        self.screen.focus(None)

        listview_id = self.footer.searching_id
        if listview_id:
            listview = self.query_one(f"#{listview_id}")
            listview.searching = False
            listview.listview.focus()
            listview.listview.index = 0

        self._searching = False

    def action_help(self) -> None:
        """Display the help modal with keyboard shortcuts."""
        self.push_screen(HelpModal())

    def action_focus_right_pane(self) -> None:
        """Focus the right pane containing tabbed content."""
        self.focused_pane = "right"

    def action_focus_next_listview(self) -> None:
        """Focus the next list view in the left pane.
        
        Cycles through node, topic, and parameter list views.
        If currently in right pane, switches to left pane.
        """
        if self._searching:
            self.end_search()
            return

        if self.focused_pane == "right":
            self.focused_pane = "left"
        else:
            self.current_pane_index = (self.current_pane_index + 1) % len(self.LISTVIEW_CONTAINERS)

    def action_focus_previous_listview(self) -> None:
        """Focus the previous list view in the left pane.
        
        Cycles backwards through node, topic, and parameter list views.
        If currently in right pane, switches to left pane.
        """
        if self._searching:
            self.end_search()
            return

        if self.focused_pane == "right":
            self.focused_pane = "left"
        else:
            self.current_pane_index = (self.current_pane_index - 1) % len(self.LISTVIEW_CONTAINERS)

    def _focus_right_pane(self) -> None:
        """Focus the currently active widget in the right pane.
        
        Focuses the rich_log widget of the currently active tab content
        if it exists.
        """
        try:
            current_listview = self.LISTVIEW_CONTAINERS[self.current_pane_index]
            tabs = self.query_one(f"#{current_listview}-tabs")
            widget = self.query_one(f"#{current_listview}-{tabs.active}-view-content")
            if hasattr(widget, "rich_log"):
                widget.rich_log.focus()
        except Exception as e:
            self.log.error(f"Error focusing right pane: {e}")

    def _focus_listview(self) -> None:
        """Focus the currently active list view in the left pane.
        
        Focuses the list view widget corresponding to the current pane index.
        """
        try:
            current_listview = self.LISTVIEW_CONTAINERS[self.current_pane_index]
            self.query_one(f"#{current_listview}-listview").listview.focus()
        except Exception as e:
            self.log.error(f"Error focusing listview: {e}")

    def _set_active_pane(self, widget: Any, active: bool) -> None:
        """Set the active state CSS class on a widget.
        
        Args:
            widget: The widget to set the active state on.
            active: Whether the widget should be marked as active.
        """
        widget.set_class(active, "-active")

    def _reset_frame_highlight(self) -> None:
        """Update visual highlighting to show the currently focused pane.
        
        Highlights the active container and removes highlighting from inactive ones.
        """
        right_active = (self.focused_pane == "right")
        self._set_active_pane(self.query_one("#right-pane"), right_active)
        active_index = self.current_pane_index if not right_active else -1
        for i, name in enumerate(self.LISTVIEW_CONTAINERS):
            w = self.query_one(f"#{name}-container")
            self._set_active_pane(w, i == active_index)

    def _update_right_pane(self) -> None:
        """Update right pane to show tabs for the currently focused list view.
        
        Hides tabs for inactive list views and shows tabs for the active one.
        """
        current_listview = self.LISTVIEW_CONTAINERS[self.current_pane_index]
        for name in self.LISTVIEW_CONTAINERS:
            tabs = self.query_one(f"#{name}-tabs")
            tabs.set_class(name != current_listview, "hidden")

    def watch_focused_pane(self, _value: str) -> None:
        """React to changes in the focused pane.
        
        Args:
            _value: The new focused pane value (unused).
        """
        self._reset_frame_highlight()
        self._focus_listview() if self.focused_pane == "left" else self._focus_right_pane()

    def watch_current_pane_index(self, _value: int) -> None:
        """React to changes in the current pane index.
        
        Args:
            _value: The new pane index value (unused).
        """
        self._reset_frame_highlight()
        self._focus_listview()
        self._update_right_pane()

    def action_previous_tab(self) -> None:
        """Navigate to the previous tab in the current tab group."""
        self._shift_tab(-1)

    def action_next_tab(self) -> None:
        """Navigate to the next tab in the current tab group."""
        self._shift_tab(+1)

    def _shift_tab(self, delta: int) -> None:
        """Shift the active tab by a given delta.
        
        Args:
            delta: The number of positions to shift (positive for next, negative for previous).
        """
        current_listview = self.LISTVIEW_CONTAINERS[self.current_pane_index]
        tabs = self.query_one(f"#{current_listview}-tabs")
        tab_ids = self.TAB_ID_DICT[current_listview]
        try:
            idx = tab_ids.index(tabs.active)
        except ValueError:
            idx = 0
        new_idx = idx + delta
        if 0 <= new_idx < len(tab_ids):
            tabs.active = tab_ids[new_idx]


def main() -> None:
    """Main entry point for the LazyROS application.
    
    Initializes the application, sets up signal handling for graceful shutdown,
    and runs the main event loop.
    """
    app = LazyRosApp()

    def _sigint(_sig: int, _frm: Any) -> None:
        """Signal handler for SIGINT (Ctrl+C).
        
        Args:
            _sig: The signal number (unused).
            _frm: The current stack frame (unused).
        """
        app.exit()

    signal.signal(signal.SIGINT, _sigint)
    try:
        app.run()
    finally:
        ros_runner.stop()


if __name__ == "__main__":
    main()
