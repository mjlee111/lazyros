import subprocess
import threading
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container
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
        width: 70%; /* Potentially wider for message definitions */
        height: 60%; /* Potentially taller */
        border: round white;
        background: $background;
        /* align: center middle; Let content flow from top */
        /* overflow-y: auto; /* Container itself doesn't scroll, content does */
    }

    #message-modal-title { /* Unique ID for title */
        dock: top;
        width: 100%;
        padding: 1;
        text-align: center;
    }
    
    #message-content-scroll-container { /* ID for the scrollable content area */
        width: 100%;
        height: 1fr; /* Fill available space */
        overflow-y: auto; /* Scroll content if it overflows */
        padding: 1;
    }

    #message-modal-content { /* Unique ID for content */
        width: 100%;
        height: auto; /* Let content determine height */
    }

    #message-modal-instruction { /* Unique ID for instruction */
        dock: bottom;
        width: 100%;
        padding: 1;
        text-align: center;
    }
    """

    BINDINGS = [
        Binding("q", "dismiss", "Quit Modal")
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
            Label(f"Message Definition: {self.message_type}", id="message-modal-title"),
            Container( # Add a container for the content to enable scrolling
                Label(self.message_info, id="message-modal-content"),
                id="message-content-scroll-container"
            ), 
            Label("Press 'q' to quit.", id="message-modal-instruction"),
            id="message-modal-container",
        )
