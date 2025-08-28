from textual.widgets import Footer
from textual.events import Key
from textual.widgets import Footer, Static
from textual.app import ComposeResult
from textual.reactive import reactive
from textual.containers import Horizontal


class SearchFooter(Footer):
    can_focus = True

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._search_mode = False
        self._buf = ""
        self.show_command_palette = False
        self.searching_id = None  # ID of the widget that initiated the search

    def compose(self):
        yield from super().compose()
        if self._search_mode:
            yield Static(f"filter: {self._buf}", id="search-filter")

    def update_buf(self) -> None:
        filter_widget = self.query_one("#search-filter")
        filter_widget.update(f"filter: {self._buf}")

    # API
    def enter_search(self) -> None:
        self._search_mode = True
        self._buf = ""

    def exit_search(self) -> None:
        self._search_mode = False
        self._buf = ""

    def on_key(self, event: Key) -> None:
        if not self._search_mode:
            return

        key = event.key

        if key == "backspace":
            self._buf = self._buf[:-1]
            self.update_buf()
        elif key in ("delete", "ctrl+u"):
            self._buf = ""
            self.update_buf()
        else:
            if len(key) == 1:
                self._buf += key
                self.update_buf()
            else:
                return

        event.stop()
