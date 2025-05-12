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

    #modal-container { /* Consistent ID with TopicInfoModal */
        width: 50%; /* Adjust size as needed, keeping it distinct from TopicInfoModal's 30% */
        height: 40%; /* Adjust size as needed */
        border: round white;
        background: $background;
        align: center middle;
    }

    #modal-message { /* Consistent ID with TopicInfoModal for title */
        margin: 1 0;
        text-align: center;
        text-style: bold;
    }

    #modal-content { /* Consistent ID with TopicInfoModal for content */
        margin: 1 2; /* Add some horizontal margin */
        height: 1fr; /* Allow content to take available space */
        overflow-y: auto; /* Enable scrolling for long content */
        width: 100%; /* Ensure content takes full width */
    }

    #modal-instruction { /* Consistent ID with TopicInfoModal */
        margin-top: 1;
        text-align: center;
    }
    """

    BINDINGS = [
        Binding("q", "dismiss", "Quit Modal")
    ]

    def __init__(self, title: str, content: str, **kwargs) -> None:
        super().__init__(**kwargs)
        self.modal_title = title
        self.modal_content = content

    def compose(self) -> ComposeResult:
        """Compose the modal dialog."""
        yield Container(
            Label(self.modal_title, id="modal-message"), # Use consistent ID
            Label(self.modal_content, id="modal-content"), # Use consistent ID
            Label("Press 'q' to quit.", id="modal-instruction"), # Use consistent ID
            id="modal-container", # Use consistent ID
            classes="modal-content",
        )

    def update_content(self, title: str, content: str) -> None:
        """Updates the title and content of the modal."""
        self.query_one("#modal-message", Label).update(title)
        self.query_one("#modal-content", Label).update(content)
