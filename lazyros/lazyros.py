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
    TabPane,
    ListView,
)

from lazyros.widgets.node.node_list import NodeListWidget
from lazyros.widgets.node.node_log import LogViewWidget
from lazyros.widgets.node.node_info import InfoViewWidget
from lazyros.widgets.node.node_lifecycle import LifecycleWidget

from lazyros.widgets.topic.topic_info import TopicInfoWidget
from lazyros.widgets.topic.topic_list import TopicListWidget
from lazyros.widgets.topic.topic_echo import EchoViewWidget

from lazyros.widgets.parameter.parameter_list import ParameterListWidget
from lazyros.widgets.parameter.parameter_value import ParameterValueWidget
from lazyros.widgets.parameter.parameter_info import ParameterInfoWidget

from textual.screen import ModalScreen
from lazyros.search import SearchFooter
from textual.events import Action


class HelpModal(ModalScreen):
    CSS = """
    HelpModal {
        align: center middle;
        layer: modal;
    }
    
    #modal-container {
        width: auto;
        height: auto;
        border: round white;
        background: $background;
    }
    """
    
    BINDINGS = [
        Binding("escape", "dismiss", "Quit Modal")
    ]
    
    def compose(self):
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

    BINDINGS = [
        Binding("q", "quit", "Quit", show=True, priority=True),
        Binding("?", "help", "Help", show=True),
        Binding("/", "search", "Search", show=False)
    ]

    CSS_PATH = "lazyros.css"
    
    NODE_TAB_ID_LIST = ["log", "info"]
    TOPIC_TAB_ID_LIST = ["echo", "info"]
    PARAMETER_TAB_ID_LIST = ["value", "info"]

    TAB_ID_DICT = {
        "node": ["log", "lifecycle", "info"],
        "topic": ["echo", "info"],
        "parameter": ["value", "info"],
    }

    def __init__(self, ros_node: Node):
        super().__init__()
        self.ros_node = ros_node

        self.left_pane_widgets = [
            "#node-listview",
            "#topic-listview", 
            "#parameter-listview"
        ]
        self._left_containers = ["node", "topic", "parameter"]
        self.current_pane_index = 0
        self.current_right_pane_config = "node"
        self.current_selected_topic = None
        self._topic_update_timer = None
        self.focused_pane = "left"

    def on_mount(self) -> None:
        """Called when app is mounted. Perform async setup here."""
        node_container = self.query_one(f'#node-container')
        if node_container:
            node_container.styles.border = ("round", "green") 

        node_list_widget = self.query_one("#node-listview")
        if node_list_widget:
            node_list_widget.listview.focus()

    def on_key(self, event) -> None:
        """Handle key events, override default tab behavior."""

        if event.key == "tab" and self.screen.focused == self.query_one(SearchFooter):
            self.deactive_search()

        if event.key == "tab":
            self.action_focus_next_listview()
            event.stop()
        elif event.key == "shift+tab":
            self.action_focus_previous_listview()
            event.stop()
        elif event.key == "enter":
            if self.screen.focused == self.query_one(SearchFooter):
                self.deactive_search(restore_focus=True, escape_searching=False)
            else:
                self.action_focus_right_pane()
            event.stop()
        elif event.character == "[":
            self.action_previous_tab()
            event.stop()
        elif event.character == "]":
            self.action_next_tab()
            event.stop()
        elif event.key == "escape":
            self.deactive_search(restore_focus=True)
            event.stop()

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""

        yield Header()

        with Horizontal():
            with Container(classes="left-pane", id="left-frame"):
                with Vertical():
                    node_container = ScrollableContainer(classes="list-container", id="node-container")
                    node_container.border_title = "Nodes"
                    with node_container:
                        yield NodeListWidget(self.ros_node, id="node-listview")

                    topic_container = ScrollableContainer(classes="list-container", id="topic-container")
                    topic_container.border_title = "Topics"
                    with topic_container:
                        yield TopicListWidget(self.ros_node, id="topic-listview")

                    parameter_container = ScrollableContainer(classes="list-container", id="parameter-container")
                    parameter_container.border_title = "Parameters"
                    with parameter_container:
                        yield ParameterListWidget(self.ros_node, id="parameter-listview")

            container = Container(classes="right-pane", id="right-frame")
            with container:
                with TabbedContent("Log", "Lifecycle", "Info", id="node-tabs"):
                    with TabPane("Log", id="log"):
                        yield LogViewWidget(self.ros_node, id="node-log-view-content")
                    with TabPane("Lifecycle", id="lifecycle"):
                        yield LifecycleWidget(self.ros_node, id="node-lifecycle-view-content")
                    with TabPane("Info", id="info"):
                        yield InfoViewWidget(self.ros_node, id="node-info-view-content")
                with TabbedContent("Info", "Echo", id="topic-tabs", classes="hidden"):
                    with TabPane("Echo", id="echo"):
                        yield EchoViewWidget(self.ros_node, id="topic-echo-view-content")
                    with TabPane("Info", id="info"):
                        yield TopicInfoWidget(self.ros_node, id="topic-info-view-content")
                with TabbedContent("Info", "Value", id="parameter-tabs", classes="hidden"):
                    with TabPane("Value", id="value"):
                        yield ParameterValueWidget(self.ros_node, id="parameter-value-view-content")
                    with TabPane("Info", id="info"):
                        yield ParameterInfoWidget(self.ros_node, id="parameter-info-view-content")

        yield SearchFooter(id="footer")

    # keybindings for actions
    def deactive_search(self, restore_focus=False, escape_searching=True) -> None:
        footer = self.query_one(SearchFooter)
        if escape_searching:
            footer.exit_search()
            self.screen.focus(None)

        if restore_focus:
            widget_id = footer.searching_id
            widget = self.query_one(f'#{widget_id}')
            widget.searching = not escape_searching 
            widget.listview.focus()

        footer.refresh(layout=True)
        self.refresh(layout=True)

    def action_search(self) -> None:
        footer = self.query_one(SearchFooter)
        focused = self.screen.focused
        #if type(focused) != ListView:
        #    self.log(f"Focused widget is not a ListView: {type(focused)}")
        #    return

        widget_id = f"{self._left_containers[self.current_pane_index]}-listview"  
        footer.searching_id = widget_id
        widget = self.query_one(f'#{widget_id}')
        widget.searching = True

        footer.enter_search()
        self.set_focus(footer)
        footer.refresh(layout=True)
        self.refresh(layout=True)

    def action_help(self):
        """Show the help modal."""

        self.push_screen(HelpModal())

    def action_focus_right_pane(self) -> None:
        """Focus the right pane and highlight it."""

        self.focused_pane = "right"
        self._reset_frame_highlight()
        self._focus_right_pane_tab()

    def action_focus_left_pane(self) -> None:
        """Focus the left pane and highlight it."""

        self.focused_pane = "left"
        self._reset_frame_highlight()
        self._focus_current_listview()

    def action_focus_next_listview(self) -> None:

        if self.focused_pane == "right":
            self.action_focus_left_pane()
        else:
            self.current_pane_index = (self.current_pane_index + 1) % len(self.left_pane_widgets)
            self._focus_current_listview()

    def action_focus_previous_listview(self) -> None:

        if self.focused_pane == "right":
            self.action_focus_left_pane()
        else:
            self.current_pane_index = (self.current_pane_index - 1) % len(self.left_pane_widgets)
            self._focus_current_listview()

    def _reset_frame_highlight(self) -> None:
        """Reset the visual style of the left and right panes."""
        
        right_pane: Container = self.query_one("#right-frame")
        
        if self.focused_pane == "right":
            right_pane.styles.border = ("round", "green")
            listview_container_widget = self.query_one(f'#{self._left_containers[self.current_pane_index]}-container')
            listview_container_widget.styles.border = ("round", "white")
        else:
            right_pane.styles.border = ("round", "white")
            for name in self._left_containers:
                widget = self.query_one(f'#{name}-container')
                if name == self._left_containers[self.current_pane_index]:
                    widget.styles.border = ("round", "green")
                else:
                    widget.styles.border = ("round", "white")

    def _focus_right_pane_tab(self):
        """Focus the right pane tab based on the current configuration."""

        left_container = self._left_containers[self.current_pane_index]
        tabs = self.query_one(f'#{left_container}-tabs')

        self.query_one(f'#{left_container}-{tabs.active}-view-content').rich_log.focus()

    def _focus_left_pane_listview(self):
        """Focus the left pane listview based on the current configuration."""
        
        focused_listview = self._left_containers[self.current_pane_index]        
        self.query_one(f'#{focused_listview}-listview').listview.focus()

    def _focus_current_listview(self) -> None:
        """Focus the current listview based on current_pane_index."""

        self._reset_frame_highlight()
        self._focus_left_pane_listview()
        self._update_right_pane()    

    def _update_right_pane(self) -> None:
        current_listview = self._left_containers[self.current_pane_index]
        
        for i in self._left_containers:
            if i == current_listview:
                self.query_one(f'#{i}-tabs').remove_class("hidden")
                self.current_right_pane_config = i
            else:
                self.query_one(f'#{i}-tabs').add_class("hidden")   

    def action_previous_tab(self):
        """Focus the right pane tab based on the current configuration."""
        
        current_listview = self._left_containers[self.current_pane_index]
        tabs = self.query_one(f'#{current_listview}-tabs')
        current_tab = tabs.active

        if self.TAB_ID_DICT[current_listview][0] == current_tab:
            return
        for i in self.TAB_ID_DICT[current_listview][1:]:
            if current_tab == i:
                tabs.active = self.TAB_ID_DICT[current_listview][self.TAB_ID_DICT[current_listview].index(i) - 1]
                break
        #self.query_one(f'#{current_listview}-{tabs.active}-view-content').rich_log.focus()

    def action_next_tab(self):
        """Focus the right pane tab based on the current configuration."""

        current_listview = self._left_containers[self.current_pane_index]
        tabs = self.query_one(f'#{current_listview}-tabs')
        current_tab = tabs.active
        
        if self.TAB_ID_DICT[current_listview][-1] == current_tab:
            return
        for i in self.TAB_ID_DICT[current_listview][:-1]:
            if current_tab == i:
                tabs.active = self.TAB_ID_DICT[current_listview][self.TAB_ID_DICT[current_listview].index(i) + 1]
                break
        #self.query_one(f'#{current_listview}-{tabs.active}-view-content').rich_log.focus()


def main(args=None):
    from lazyros.utils.utility import start_ros_in_thread, stop_ros_thread
    rclpy.init(args=args)
    ros_node = Node("lazyros_monitor_node")
    executor, ros_thread = start_ros_in_thread(ros_node)
    app = LazyRosApp(ros_node)
    app.run()

if __name__ == "__main__":
    main()
