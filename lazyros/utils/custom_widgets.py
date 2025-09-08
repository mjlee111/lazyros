import os
from textual.events import Focus, Key
from textual.widgets import (
    ListView,
    RichLog
)
from textual.binding import Binding
from textual.widgets import Header, Footer, Static
from textual.reactive import Reactive


class CustomListView(ListView):
    """Custom ListView that automatically focuses on mount."""

    def on_focus(self, event: Focus) -> None:
        if self.children and not self.index:
            self.index = 0

    def on_key(self, event: Key) -> None:
        if event.key in ("up", "down"):
            items = [i for i in self.children if i.display] 
            if not items:
                return

            current = self.index or 0
            if current >= len(self.children) or not self.children[current].display:
                self.index = self.children.index(items[0])
                return

            if event.key == "down":
                visible_next = next((i for i in items if self.children.index(i) > current), None)
                if visible_next:
                    self.index = self.children.index(visible_next)
            elif event.key == "up":
                visible_prev = next((i for i in reversed(items) if self.children.index(i) < current), None)
                if visible_prev:
                    self.index = self.children.index(visible_prev)

            event.stop()


class CustomRichLog(RichLog):
    """Custom RichLog with additional key bindings for navigation."""

    BINDINGS = [
        Binding("g,g", "go_top", "Top", show=False),     # gg -> 先頭へ
        Binding("G", "go_bottom", "Bottom", show=False), # G  -> 末尾へ
        Binding("j", "scroll_down", "Down", show=False), # 1行下
        Binding("k", "scroll_up", "Up", show=False),     # 1行上
    ]

    def action_go_top(self) -> None:
        super().action_scroll_home()
        self.auto_scroll = False

    def action_go_bottom(self) -> None:
        super().action_scroll_end()
        self.auto_scroll = True

    def action_scroll_up(self) -> None:
        super().action_scroll_up()
        self.auto_scroll = False

    def action_scroll_down(self) -> None:
        super().action_scroll_down()
        self.auto_scroll = False


class SearchFooter(Footer):
    """Footer with search functionality."""

    can_focus = True
    search_input = Reactive("")

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.input = ""
        self.show_command_palette = False
        self.searching_id = None
        self._searching = False

    def compose(self):
        self.search_widget = Static(f"filter: {self.search_input}", id="search-filter")
        if self._searching:
            yield self.search_widget
        else:
            yield from super().compose()

    def watch_search_input(self, value: str) -> None:
        if self._searching:
            self._update_buf()

    def _update_buf(self) -> None:
        self.search_widget.update(f"filter: {self.search_input}")
        
    def _clear_input(self) -> None:
        self.search_input = ""

    def enter_search(self) -> None:
        self._searching = True
        self._clear_input()

    def exit_search(self) -> None:
        self._searching = False
        self._clear_input()

    def on_key(self, event: Key) -> None:
        if not self._searching:
            return

        key = event.key
        if key == "backspace":
            self.search_input = self.search_input[:-1]
        elif key in ("delete", "ctrl+u"):
            self.search_input = ""
        else:
            if len(key) == 1:
                self.search_input += key
            else:
                return
        event.stop()


class CustomHeader(Header):
    def __init__(self, **kwargs):
        ros_distro = os.environ.get("ROS_DISTRO", "unknown")
        ros_domain = os.environ.get("ROS_DOMAIN_ID", "0")
        dds_implementation = os.environ.get("RMW_IMPLEMENTATION", "unknown")
        title = f"LazyROS  |  ROS_DISTRO={ros_distro}  |  ROS_DOMAIN_ID={ros_domain}  |  DDS_IMPLEMENTATION={dds_implementation}"
        super().__init__(show_clock=True, **kwargs)
