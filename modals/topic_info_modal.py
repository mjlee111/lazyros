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
        layer: modal; /* Ensure it appears above the main screen */
    }

    #modal-container {
        width: 30%;
        height: 30%;
        border: round white;
        background: $background;
        align: center middle;
    }

    #modal-message {
        margin: 1 0;
        text-align: center;
    }

    #modal-instruction {
        margin-top: 1;
        text-align: center;
    }
    """

    BINDINGS = [
        Binding("q", "dismiss", "Quit Modal")
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
            Label(f"Topic Information: {escape_markup(self.topic_name)}", id="modal-message"),
            Label(self.topic_info, id="modal-content"), # topic_info is already escaped
            Label("Press 'q' to quit.", id="modal-instruction"),
            id="modal-container",
            classes="modal-content",
        )
