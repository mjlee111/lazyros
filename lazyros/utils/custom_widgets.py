import os
from typing import Any, Optional

from textual.app import ComposeResult
from textual.binding import Binding
from textual.events import Focus, Key
from textual.reactive import Reactive
from textual.widgets import Footer, Header, ListView, RichLog, Static


class CustomListView(ListView):
    """Custom ListView that automatically focuses on mount and handles keyboard navigation."""

    def on_focus(self, event: Focus) -> None:
        """Handle focus events by setting initial index if none exists.
        
        Args:
            event (Focus): The focus event
        """
        try:
            if self.children and not self.index:
                self.index = 0
        except Exception:
            pass

    def on_key(self, event: Key) -> None:
        """Handle keyboard navigation for visible items only.
        
        Args:
            event (Key): The key event
        """
        try:
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
        except Exception:
            pass


class CustomRichLog(RichLog):
    """Custom RichLog with additional key bindings for navigation."""

    BINDINGS = [
        Binding("g,g", "go_top", "Top", show=False),     # gg -> 先頭へ
        Binding("G", "go_bottom", "Bottom", show=False), # G  -> 末尾へ
        Binding("j", "scroll_down", "Down", show=False), # 1行下
        Binding("k", "scroll_up", "Up", show=False),     # 1行上
    ]

    def action_go_top(self) -> None:
        """Navigate to the top of the log.
        
        Scrolls to the beginning of the log content and disables auto-scroll.
        """
        super().action_scroll_home()
        self.auto_scroll = False

    def action_go_bottom(self) -> None:
        """Navigate to the bottom of the log.
        
        Scrolls to the end of the log content and enables auto-scroll.
        """
        super().action_scroll_end()
        self.auto_scroll = True

    def action_scroll_up(self) -> None:
        """Scroll up one line in the log.
        
        Scrolls up by one line and disables auto-scroll.
        """
        super().action_scroll_up()
        self.auto_scroll = False

    def action_scroll_down(self) -> None:
        """Scroll down one line in the log.
        
        Scrolls down by one line and disables auto-scroll.
        """
        super().action_scroll_down()
        self.auto_scroll = False


class SearchFooter(Footer):
    """Footer with search functionality."""

    can_focus = True
    search_input = Reactive("")

    def __init__(self, **kwargs: Any) -> None:
        """Initialize the SearchFooter.
        
        Args:
            **kwargs: Additional keyword arguments passed to the parent Footer class
        """
        super().__init__(**kwargs)
        self.input = ""
        self.show_command_palette = False
        self.searching_id = None
        self._searching = False

    def compose(self) -> ComposeResult:
        """Compose the footer content based on search state.
        
        Returns:
            ComposeResult: The widgets to display in the footer
        """
        self.search_widget = Static(f"filter: {self.search_input}", id="search-filter")
        if self._searching:
            yield self.search_widget
        else:
            yield from super().compose()

    def watch_search_input(self, value: str) -> None:
        """Watch for changes to the search input and update display.
        
        Args:
            value (str): The new search input value
        """
        if self._searching:
            self._update_buf()

    def _update_buf(self) -> None:
        """Update the search widget display with current input."""
        self.search_widget.update(f"filter: {self.search_input}")
        
    def _clear_input(self) -> None:
        """Clear the search input."""
        self.search_input = ""

    def enter_search(self) -> None:
        """Enter search mode and clear any existing input."""
        self._searching = True
        self._clear_input()

    def exit_search(self) -> None:
        """Exit search mode and clear any existing input."""
        self._searching = False
        self._clear_input()

    def on_key(self, event: Key) -> None:
        """Handle key events for search input.
        
        Args:
            event (Key): The key event to handle
        """
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
    """Custom Header that displays ROS environment information in the title."""

    def __init__(self, **kwargs: Any) -> None:
        """Initialize the CustomHeader with ROS environment information.
        
        Args:
            **kwargs: Additional keyword arguments passed to the parent Header class
        """
        ros_distro = os.environ.get("ROS_DISTRO", "unknown")
        ros_domain = os.environ.get("ROS_DOMAIN_ID", "0")
        dds_implementation = os.environ.get("RMW_IMPLEMENTATION", "unknown")
        title = f"LazyROS  |  ROS_DISTRO={ros_distro}  |  ROS_DOMAIN_ID={ros_domain}  |  DDS_IMPLEMENTATION={dds_implementation}"
        super().__init__(show_clock=True, **kwargs)
