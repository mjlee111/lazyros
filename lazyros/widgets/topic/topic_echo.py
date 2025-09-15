import time
from collections import deque
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from lazyros.utils.ros_compatibility import create_qos_profile, create_callback_group
from rich.markup import escape
from rosidl_runtime_py.utilities import get_message
from textual.app import ComposeResult
from textual.containers import Container

from lazyros.utils.custom_widgets import CustomRichLog 


class EchoViewWidget(Container):
    """Widget for displaying ROS topic echo messages."""

    DEFAULT_CSS = """
    EchoViewWidget {
        overflow-y: scroll;
    }
    """

    def __init__(self, ros_node: Node, **kwargs: Any) -> None:
        """Initialize the EchoViewWidget.
        
        Args:
            ros_node: The ROS node instance for communication
            **kwargs: Additional keyword arguments passed to the parent Container
        """
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.current_topic = None

        self.topic_listview = None
        self.echo_dict = None
        self._sub = None
        self._buffer = []
        self.rich_log = CustomRichLog(wrap=True, highlight=True, markup=True, id="echo-log", max_lines=1000)
        self._prev_echo_time = time.time()

        self.callback_group = create_callback_group()

    def compose(self) -> ComposeResult:
        """Compose the widget layout.
        
        Returns:
            ComposeResult: A generator yielding widget components
        """
        yield self.rich_log

    def on_mount(self) -> None:
        """Handle the widget mount event.
        
        Sets up periodic interval to update display.
        """
        self.set_interval(1, self.update_display)

    def update_display(self) -> None:
        """Update the echo display with messages from the selected topic.
        
        Switches topics when selection changes and displays buffered messages.
        """
        try:
            self.topic_listview = self.app.query_one("#topic-listview")
            self.selected_topic = self.topic_listview.selected_topic if self.topic_listview else None

            if self.selected_topic is None:
                self._clear_log()
                self.rich_log.write("[red]Please select a topic first.[/]")
                return

            if self.selected_topic == self.current_topic:
                if len(self._buffer) > 0:
                    self.rich_log.write("\n".join(self._buffer))
                    self._clear_buffer()
                    self._prev_echo_time = time.time()
                else:
                    if time.time() - self._prev_echo_time> 5.0:
                        self.rich_log.write(f"[yellow]No messages received yet (checking every 5 seconds).[/yellow]")
                        self._prev_echo_time = time.time()
                return

            self.current_topic = self.selected_topic

            self.rich_log.clear()
            self._clear_buffer()
            self._prev_echo_time = time.time()

            self._switch_topic_and_subscribe()
        except Exception:
            pass


    def _clear_buffer(self) -> None:
        """Clear the message buffer."""
        self._buffer = []

    def _clear_log(self) -> None:
        """Clear both the message buffer and log display."""
        self._clear_buffer()
        self.rich_log.clear()

    def _switch_topic_and_subscribe(self) -> None:
        """Switch to a new topic and create subscription.
        
        Destroys existing subscription and creates a new one for the current topic.
        """
        if self._sub is not None:
            try:
                if hasattr(self._sub, 'handle') and self._sub.handle is not None:
                    self.ros_node.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = None

        topic_dict = self.topic_listview.topic_dict if self.topic_listview else {}
        type_list = topic_dict.get(self.current_topic, None)
        if not type_list:
            self.rich_log.write(f"[red]Topic '{escape(self.current_topic)}' is not available.[/]")
            return

        topic_type = type_list[0]
        try:
            msg_type = get_message(topic_type)
        except Exception as e:
            self.rich_log.write(f"[red]Unable to determine message type for '{escape(self.current_topic)}': "
                        f"{escape(str(e))}[/]")
            return

        self.rich_log.write(
            f"[bold]Monitoring topic: [yellow]{escape(self.current_topic)}[/] "
            f"[dim]({escape(topic_type)})[/][/bold]"
        )

        qos_profile = create_qos_profile(depth=100)
        try:
            self._sub = self.ros_node.create_subscription(
                msg_type, self.current_topic, self.echo_callback,
                qos_profile=qos_profile, callback_group=self.callback_group
            )

        except Exception as e:
            self.rich_log.write(f"[red]Unable to subscribe to '{escape(self.current_topic)}': "
                        f"{escape(str(e))}[/]")
            self._sub = None
            return

    def echo_callback(self, msg: Any) -> None:
        """Callback function for receiving topic messages.
        
        Args:
            msg: The received message from the subscribed topic
        """
        try:
            if not self._sub:
                return
            line = f"[dim]Message from {escape(self.current_topic)}: [/dim] {escape(str(msg))}"
            self._buffer.append(line)
        except Exception:
            pass
