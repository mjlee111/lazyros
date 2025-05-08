import subprocess
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import Label
from textual.screen import ModalScreen
from rich.markup import escape as escape_markup


class TopicInfoModal(ModalScreen):
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
        ("q", "dismiss", "Quit Modal")
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


class MessageModal(ModalScreen):
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
        align: center middle;
        overflow-y: auto; /* Allow scrolling for long message definitions */
    }

    #message-modal-title { /* Unique ID for title */
        margin: 1 0;
        text-align: center;
        font-weight: bold;
    }
    
    #message-modal-content { /* Unique ID for content */
        margin: 1;
        width: 100%;
        height: auto; /* Let content determine height */
        overflow-y: auto; /* Scroll content if it overflows */
    }

    #message-modal-instruction { /* Unique ID for instruction */
        margin-top: 1;
        text-align: center;
    }
    """

    BINDINGS = [
        ("q", "dismiss", "Quit Modal")
    ]

    def __init__(self, message_type: str, **kwargs):
        super().__init__(**kwargs)
        self.message_type = message_type
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
        except Exception as e:
            return escape_markup(f"Unexpected error for {self.message_type}: {str(e)}")

    def compose(self) -> ComposeResult:
        """Compose the modal dialog."""
        yield Container(
            Label(f"Message Definition: {escape_markup(self.message_type)}", id="message-modal-title"),
            Container( # Add a container for the content to enable scrolling
                Label(self.message_info, id="message-modal-content"),
                id="message-content-scroll-container" # ID for the scrollable content area
            ), 
            Label("Press 'q' to quit.", id="message-modal-instruction"),
            id="message-modal-container", # Use the unique ID
        )
