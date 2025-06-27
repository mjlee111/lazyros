import rclpy
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container, VerticalScroll
from textual.widgets import Label, Button
from textual.screen import ModalScreen

from lifecycle_msgs.srv import GetAvailableTransitions, ChangeState, GetState
from lifecycle_msgs.msg import Transition
from typing import List

class LifecycleModal(ModalScreen[None]):
    """A modal screen to display node information."""

    CSS = """
    LifecycleModal {
        align: center middle;
        layer: modal;
    }

    #lifycycle-modal-container {
        width: 40%;
        height: auto;
        border: round white;
        background: $background;
        overflow-y: auto;
    }

    #lifycycle-modal-title {
        dock: top;
        width: 100%;
        text-align: center;
        padding: 1;
        background: $primary-background-darken-1;
    }

    #lifycycle-modal-content {
        width: 100%;
        border: round $primary;
        margin: 0 1;
        overflow-y: auto;
    }

    #lifycycle-modal-instruction {
        dock: bottom;
        width: 100%;
        text-align: center;
        padding: 1;
    }

    Button {
        width: 30%;
        margin: 1 2;
        text-align: center;
        background: $primary;
        color: $text;
        border: round $primary;
        height: 3;
    }
    """

    BINDINGS = [
        Binding("escape", "dismiss", "Quit Modal")
    ]

    def __init__(self, node, node_data, **kwargs):
        super().__init__(**kwargs)
        self.node = node
        self.node_name = node_data.name
        self.status = node_data.status
        self.is_lifecycle = node_data.is_lifecycle

        self.get_state_client = self.node.create_client(GetState, f"{self.node_name}/get_state")
        self.get_transition_client = self.node.create_client(GetAvailableTransitions, f"{self.node_name}/get_available_transitions")
        self.change_state_client = self.node.create_client(ChangeState, f"{self.node_name}/change_state")

    def on_mount(self) -> None:
        self.update_display()

    def update_display(self) -> None:
        self.node.get_logger().info(f"Updating display for node: {self.node_name}")
        title = f"Lifecycle Info: {self.node_name}"
        self.query_one("#lifycycle-modal-title").update(title)

        content = ""
        
        if self.status != "green":
            content = "Node is not running.\n"
            self.query_one("#lifycycle-modal-text").update(content) # Changed selector
            return
        if not self.is_lifecycle:
            content = "This is not a lifecycle node.\n"
            self.query_one("#lifycycle-modal-text").update(content) # Changed selector
            return
        
        current_state = self.get_current_state()
        content += f"Lifecycle State: {current_state}\n\n"
        transitions = self.get_available_transitions()
        if transitions:
            content += "Available Transitions:\n"
            # Remove existing transition buttons
            for button in self.query(Button):
                if button.id and button.id.startswith("transition-button-"):
                    button.remove()
            for transition in transitions:
                content += f"- {transition.transition.label}\n"
                # Add a button for each transition
                self.query_one("#lifycycle-modal-content").mount(
                    Button(transition.transition.label, id=f"transition-button-{transition.transition.id}")
                )
        else:
            content += "No available transitions."
            for button in self.query(Button):
                button.remove()

        self.query_one("#lifycycle-modal-text").update(content) # Changed selector

    def get_current_state(self):
        while not self.get_state_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('get_state service not available, waiting again...')
        
        request = GetState.Request()
        future = self.get_state_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            return future.result().current_state.label
        else:
            self.node.get_logger().error('Exception while calling get_state service: %r' % future.exception())
            return "Unknown"

    def get_available_transitions(self):
        while not self.get_transition_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('get_available_transitions service not available, waiting again...')

        request = GetAvailableTransitions.Request()
        future = self.get_transition_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            return future.result().available_transitions 
        else:
            self.node.get_logger().error('Exception while calling get_available_transitions service: %r' % future.exception())
            return []

    def trigger_transition(self, transition_id: int):
        while not self.change_state_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('change_state service not available, waiting again...')

        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.change_state_client.call_async(request)
        future.add_done_callback(self.transition_callback)

    def transition_callback(self, future):
        if future.result() is not None:
            if future.result().success:
                self.node.get_logger().info(f"Transition successful for {self.node_name}")
                if self.app:
                    self.app.call_from_thread(self.update_display) # Update display in the main thread
            else:
                self.node.get_logger().error(f"Transition failed for {self.node_name}")
        else:
            self.node.get_logger().error('Exception while calling service: %r' % future.exception())
    
    def compose(self):
        yield Container(
            Label("", id="lifycycle-modal-title"),
            VerticalScroll(
                Label("", id="lifycycle-modal-text"),
                id="lifycycle-modal-content",
            ),
            Label("Press 'ESC' to quit.", id="lifycycle-modal-instruction"),
            id="lifycycle-modal-container",
        )

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id.startswith("transition-button-"):
            transition_id = int(event.button.id.split("-")[-1])
            self.trigger_transition(transition_id)

    def on_dismiss(self) -> None:
        super().on_dismiss()
