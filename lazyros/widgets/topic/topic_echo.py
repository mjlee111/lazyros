import subprocess
import threading
import time

from rclpy.node import Node
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import RichLog
from rich.markup import escape
from rich.text import Text as RichText
from rclpy.callback_groups import ReentrantCallbackGroup


def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

class EchoViewWidget(Container):
    """Widget for displaying ROS topic echo messages."""

    DEFAULT_CSS = """
    EchoViewWidget {
        overflow-y: scroll;
    }
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.echo_log = RichLog(wrap=True, highlight=True, markup=True, id="echo-log", max_lines=1000)
        self.current_topic = None
        self.echo_process = None
        self.echo_thread = None
        self.is_echoing = False
        self.update_rate = 2.0  # Display updates per second (Hz)
        self.last_update_time = 0

        self.topic_listview = None
        self.echo_dict = None

        self.ros_node.create_timer(1, self.update_display, callback_group=ReentrantCallbackGroup())

    def compose(self) -> ComposeResult:
        yield self.echo_log

    def update_display(self):
        self.topic_listview = self.app.query_one("#topic-listview")
        self.selected_topic = self.topic_listview.selected_topic if self.topic_listview else None

        if self.selected_topic is None:
            self.info_log.clear()
            self.info_log.write("[red]No topic is selected yet.[/]")
            return

        if self.selected_topic == self.current_topic:
            return

        self.current_topic = self.selected_topic
        self.echo_log.clear()
        self.start_echo()

    def start_echo(self):
        self.destroy_subscription(self._sub)

        topic_dict = self.topic_listview.topic_dict if self.topic_listview else None
        if not topic_dict:
            return [f"[red]Topic {escape_markup(self.current_topic)} is not set.[/]"]

        topic_type = topic_dict.get(self.current_topic, None)
        self._sub = self.create_subscription(topic_type, self.current_topic, self.echo_callback)

    def echo_callback(self, msg):
        message = f"[dim]Message from {escape_markup(self.current_topic)}: [/dim] {escape_markup(str(msg.data))}"
        self.echo_log.write(message)
