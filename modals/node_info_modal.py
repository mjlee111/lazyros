from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container, VerticalScroll
from textual.widgets import Label
from textual.screen import ModalScreen

class NodeInfoModal(ModalScreen[None]):
    """A modal screen to display node information."""

    CSS = """
    NodeInfoModal {
        align: center middle;
        layer: modal;
    }

    #node-info-modal-container {
        width: 40%;
        height: 30%;
        border: round white;
        background: $background;
        overflow-y: auto;
    }

    #node-info-modal-title {
        dock: top;
        width: 100%;
        text-align: center;
        padding: 1;
        background: $primary-background-darken-1;
    }

    #node-info-modal-content {
        width: 100%;
        border: round $primary;
        margin: 0 1;
        overflow-y: auto;
    }

    #node-info-modal-instruction {
        dock: bottom;
        width: 100%;
        text-align: center;
        padding: 1;
    }
    """

    BINDINGS = [
        Binding("q", "dismiss", "Quit Modal")
    ]

    def __init__(self, node_name: str, is_lifecycle: bool, lifecycle_state: str = None, **kwargs):
        super().__init__(**kwargs)
        self.node_name = node_name
        self.is_lifecycle = is_lifecycle
        self.lifecycle_state = lifecycle_state

    def compose(self) -> ComposeResult:
        """Compose the modal dialog."""
        title = f"Node Info: {self.node_name}"
        if self.is_lifecycle:
            content = f"Lifecycle State: {self.lifecycle_state}"
        else:
            content = "This is not a lifecycle node."

        yield Container(
            Label(title, id="node-info-modal-title"),
            VerticalScroll(
                Label(content, id="node-info-modal-content"),
            ),
            Label("Press 'q' to quit.", id="node-info-modal-instruction"),
            id="node-info-modal-container",
        )
