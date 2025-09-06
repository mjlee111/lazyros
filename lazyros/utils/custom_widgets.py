from textual.events import Focus, Key
from textual.widgets import (
    ListView,
    RichLog
)
from textual.binding import Binding

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
            if not self.children[current].display:
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