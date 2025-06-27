import subprocess
import threading
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container, VerticalScroll
from textual.widgets import Label, Log
from textual.screen import ModalScreen
from rich.markup import escape as escape_markup


class MessageModal(ModalScreen[None]):
    """A modal screen to display ROS message type information."""

    CSS = """
    MessageModal {
        align: center middle;
        layer: modal;
    }

    #message-modal-container { /* Unique ID for this modal's container */
        width: 40%;
        height: 30%;
        border: round white;
        background: $background;
        overflow-y: auto;
    }

    #message-modal-title { /* Unique ID for title */
        dock: top;
        width: 100%;
        text-align: center;
        padding: 1;
        background: $primary-background-darken-1;
    }

    #message-modal-content {
        width: 100%;
        border: round $primary;
        margin: 0 1;
        overflow-y: auto; /* Enable scrolling for content */
    }

    #message-modal-instruction {
        dock: bottom;
        width: 100%;
        text-align: center;
        padding: 1;
    }
    """

    BINDINGS = [
        Binding("escape", "dismiss", "Quit Modal")
    ]

    def __init__(self, message_type: str, **kwargs):
        super().__init__(**kwargs)
        self.message_type = message_type.strip()
        self.message_info = self.get_message_info()

    def get_message_info(self) -> str:
        """Fetch the message type information using `ros2 interface show` command."""
        try:
            result = subprocess.run(
                ["ros2", "interface", "show", self.message_type],
                capture_output=True,
                text=True,
                check=True
            )
            return escape_markup(result.stdout)
        except subprocess.CalledProcessError as e:
            return escape_markup(f"Error fetching message info for {self.message_type}: {e.stderr.strip()}")
        except FileNotFoundError:
            return escape_markup(f"Error: 'ros2' command not found. Is ROS2 installed and sourced?")
        except Exception as e:
            return escape_markup(f"Unexpected error for {self.message_type}: {str(e)}")

    def compose(self) -> ComposeResult:
        """Compose the modal dialog."""
        yield Container(
            Label(f"ROS Message: {self.message_type}", id="message-modal-title"),
            VerticalScroll(  # Ensure the content is scrollable
                Label(self.message_info, id="message-modal-content"),
            ),
            Label("Press 'ESC' to quit.", id="message-modal-instruction"),
            id="message-modal-container",
        )
