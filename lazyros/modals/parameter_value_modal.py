import subprocess
import threading
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container
from textual.widgets import Label, Log
from textual.screen import ModalScreen
from rich.markup import escape as escape_markup


class ParameterValueModal(ModalScreen[None]):
    """A modal screen to display a parameter's value."""

    CSS = """
    ParameterValueModal {
        align: center middle;
        layer: modal; /* Ensure it appears above the main screen */
    }
    
    #modal-title {
        dock: top;
        width: 100%;
        text-align: center;
        padding: 1;
        background: $primary-background-darken-1;
    }
    
    #modal-container { /* Consistent ID with TopicInfoModal */
        width: 40%; /* Adjust size as needed, keeping it distinct from TopicInfoModal's 30% */
        height: auto; /* Adjust size as needed */
        border: round white;
        background: $background;
        align: center middle;
    }

    #modal-content { /* Consistent ID with TopicInfoModal for content */
        width: 100%;
        margin-top: 1; /* Add top margin for spacing */
        overflow-y: auto; /* Enable scrolling for content */
    }

    #modal-instruction { /* Consistent ID with TopicInfoModal */
        dock: bottom;
        width: 100%;
        text-align: center;
        padding: 1;
    }
    """

    BINDINGS = [
        Binding("escape", "dismiss", "Quit Modal")
    ]

    def __init__(self, title: str, content: str, **kwargs) -> None:
        super().__init__(**kwargs)
        self.modal_title = title
        self.modal_content = content

    def compose(self) -> ComposeResult:
        """Compose the modal dialog."""
        yield Container(
            Label(self.modal_title, id="modal-title"),
            Label(self.modal_content, id="modal-content"),
            Label("Press 'ESC' to quit.", id="modal-instruction"),
            id="modal-container",
        )

    def update_content(self, title: str, content: str) -> None:
        """Updates the title and content of the modal."""
        self.query_one("#modal-title", Label).update(title)
        self.query_one("#modal-content", Label).update(content)
