import subprocess
import threading
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container
from textual.widgets import Label, Log
from textual.screen import ModalScreen
from rich.markup import escape as escape_markup


class TopicInfoModal(ModalScreen[None]):
    """A modal screen to display topic information."""

    CSS = """
    TopicInfoModal {
        align: center middle;
        layer: modal;
    }

    #modal-container {
        width: 40%;
        height: auto;
        border: round white;
        background: $background;
    }
    
    #modal-title {
        dock: top;
        width: 100%;
        text-align: center;
        padding: 1;
        background: $primary-background-darken-1;
    }

    #modal-content {
        width: 100%;
        height: auto;
        border: round $primary;
        margin: 0 1;
    }

    #modal-instruction {
        dock: bottom;
        width: 100%;
        text-align: center;
        padding: 1;
    }
    """

    BINDINGS = [
        Binding("escape", "dismiss", "Quit Modal")
    ]

    def __init__(self, topic_name: str, **kwargs):
        super().__init__(**kwargs)
        self.topic_name = topic_name
        self.topic_info = self.get_topic_info()

    def get_topic_info(self) -> str:
        """Fetch the topic information using `ros2 topic info` command."""
        try:
            result = subprocess.run(
                ["ros2", "topic", "info", self.topic_name],
                capture_output=True,
                text=True,
                check=True
            )
            return escape_markup(result.stdout) # Escape markup for safety
        except subprocess.CalledProcessError as e:
            return escape_markup(f"Error fetching topic info: {e.stderr.strip()}")
        except FileNotFoundError:
            return escape_markup(f"Error: 'ros2' command not found. Is ROS2 installed and sourced?")
        except Exception as e:
            return escape_markup(f"Unexpected error: {str(e)}")

    def compose(self) -> ComposeResult:
        """Compose the modal dialog."""
        yield Container(
            Label(f"Topic Information: {escape_markup(self.topic_name)}", id="modal-title"),
            Label(self.topic_info, id="modal-content"), # topic_info is already escaped
            Label("Press 'ESC' to quit.", id="modal-instruction"),
            id="modal-container",
            classes="modal-content",
        )
