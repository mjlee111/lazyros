import signal
import atexit

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal, Vertical, ScrollableContainer
from textual.widgets import Header, Static, TabbedContent, TabPane
from textual.reactive import reactive
from textual.screen import ModalScreen

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

from lazyros.utils.custom_widgets import SearchFooter
from lazyros.utils.utility import RosRunner


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

    def compose(self):
        help_text = (
            "Help Menu\n"
            "\n"
            "enter        Focus right window\n"
            "[            Previous Tab\n"
            "]            Next Tab\n"
            "tab          Focus next container\n"
            "shift+tab    Focus previous container"
        )
        yield Static(help_text, id="modal-container")


# ===================== Textual App =====================
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

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self._searching = False
        ros_runner.start()
        self.ros_node = ros_runner.node
        assert self.ros_node is not None, "ROS node must be available before compose()"

    def on_mount(self) -> None:
        node_list_widget = self.query_one("#node-listview")
        node_list_widget.listview.focus()
        self.right_pane = self.query_one("#right-pane")

    def on_shutdown(self, _event) -> None:
        ros_runner.stop()

    def on_key(self, event) -> None:
        #if event.key == "tab" and self._searching:
        #    self.end_search()
        #    event.stop()
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
        yield Header()

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

            container = Container(classes="right-pane", id="right-pane")
            with container:
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

    # ========= actions =========

    def action_search(self):
        self._searching = True 
        searching_listview = self.LISTVIEW_CONTAINERS[self.current_pane_index] + "-listview"
        self.footer.searching_id = searching_listview
        listview = self.query_one(f"#{searching_listview}")
        listview.searching = True

        self.footer.enter_search()
        self.set_focus(self.footer)

    def focus_searched_listview(self):
        listview_id = self.footer.searching_id
        if listview_id:
            listview = self.query_one(f"#{listview_id}")
            listview.listview.focus()
        
    def end_search(self):
        self.footer.exit_search()
        self.screen.focus(None)

        listview_id = self.footer.searching_id
        if listview_id:
            listview = self.query_one(f"#{listview_id}")
            listview.searching = False
            listview.listview.focus()

        self._searching = False

        self.footer.refresh(layout=True)
        self.refresh(layout=True)

    #def deactive_search(self, focus_listview=False, end_searching=True) -> None:
    #
    #    footer = self.query_one(SearchFooter)
    #    if end_searching:
    #        footer.exit_search()
    #        self.screen.focus(None)
#
    #    # focus back to listview during searching
    #    if focus_listview:
    #        widget_id = footer.searching_id
    #        widget = self.query_one(f"#{widget_id}")
    #        widget.searching = not end_searching 
    #        widget.listview.focus()

    def action_help(self):
        self.push_screen(HelpModal())

    def action_focus_right_pane(self) -> None:
        self.focused_pane = "right"

    def action_focus_next_listview(self) -> None:
        if self._searching:
            self.end_search()
            return

        if self.focused_pane == "right":
            self.focused_pane = "left"
        else:
            self.current_pane_index = (self.current_pane_index + 1) % len(self.LISTVIEW_CONTAINERS)

    def action_focus_previous_listview(self) -> None:
        if self._searching:
            self.end_search()
            return

        if self.focused_pane == "right":
            self.focused_pane = "left"
        else:
            self.current_pane_index = (self.current_pane_index - 1) % len(self.LISTVIEW_CONTAINERS)

    # ========= helpers =========
    def _focus_right_pane(self):
        current_listview = self.LISTVIEW_CONTAINERS[self.current_pane_index]
        tabs = self.query_one(f"#{current_listview}-tabs")
        widget = self.query_one(f"#{current_listview}-{tabs.active}-view-content")
        if hasattr(widget, "rich_log"):
            widget.rich_log.focus()

    def _focus_listview(self):
        current_listview = self.LISTVIEW_CONTAINERS[self.current_pane_index]
        self.query_one(f"#{current_listview}-listview").listview.focus()

    def _set_active_pane(self, widget, active: bool) -> None:
        widget.set_class(active, "-active")

    def _reset_frame_highlight(self) -> None:
        right_active = (self.focused_pane == "right")
        self._set_active_pane(self.query_one("#right-pane"), right_active)
        active_index = self.current_pane_index if not right_active else -1
        for i, name in enumerate(self.LISTVIEW_CONTAINERS):
            w = self.query_one(f"#{name}-container")
            self._set_active_pane(w, i == active_index)

    def _update_right_pane(self) -> None:
        current_listview = self.LISTVIEW_CONTAINERS[self.current_pane_index]
        for name in self.LISTVIEW_CONTAINERS:
            tabs = self.query_one(f"#{name}-tabs")
            tabs.set_class(name != current_listview, "hidden")

    def watch_focused_pane(self, _value) -> None:
        self._reset_frame_highlight()
        self._focus_listview() if self.focused_pane == "left" else self._focus_right_pane()

    def watch_current_pane_index(self, _value) -> None:
        self._reset_frame_highlight()
        self._focus_listview()
        self._update_right_pane()

    def action_previous_tab(self) -> None:
        self._shift_tab(-1)

    def action_next_tab(self) -> None:
        self._shift_tab(+1)

    def _shift_tab(self, delta: int) -> None:
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


def main():
    app = LazyRosApp()

    def _sigint(_sig, _frm):
        app.exit()

    signal.signal(signal.SIGINT, _sigint)
    try:
        app.run()
    finally:
        ros_runner.stop()


if __name__ == "__main__":
    main()
