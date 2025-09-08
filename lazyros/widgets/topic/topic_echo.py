import time
import rclpy
from collections import deque
from rclpy.node import Node
from textual.app import ComposeResult
from textual.containers import Container
from rich.markup import escape
from rclpy.callback_groups import ReentrantCallbackGroup
from rosidl_runtime_py.utilities import get_message
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile
from lazyros.utils.custom_widgets import CustomRichLog 


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
        self.current_topic = None

        self.topic_listview = None
        self.echo_dict = None
        self._sub = None
        self._buffer = []
        self.rich_log = CustomRichLog(wrap=True, highlight=True, markup=True, id="echo-log", max_lines=1000)
        self._prev_echo_time = time.time()

        self.callback_group = ReentrantCallbackGroup()

    def compose(self) -> ComposeResult:
        yield self.rich_log

    def on_mount(self):
        self.set_interval(1, self.update_display)

    def update_display(self):
        self.topic_listview = self.app.query_one("#topic-listview")
        self.selected_topic = self.topic_listview.selected_topic if self.topic_listview else None

        if self.selected_topic is None:
            self._clear_log()
            self.rich_log.write("[red]No topic is selected yet.[/]")
            return

        if self.selected_topic == self.current_topic:
            if len(self._buffer) > 0:
                self.rich_log.write("\n".join(self._buffer))
                self._clear_buffer()
                self._prev_echo_time = time.time()
            else:
                if time.time() - self._prev_echo_time> 5.0:
                    self.rich_log.write(f"[yellow]No messages on this topic yet. (checks every 5 sec)[/yellow]")
                    self._prev_echo_time = time.time()
            return

        self.current_topic = self.selected_topic

        self.rich_log.clear()
        self._clear_buffer()
        self._prev_echo_time = time.time()

        self._switch_topic_and_subscribe()


    def _clear_buffer(self) -> None:
        self._buffer = []

    def _clear_log(self) -> None:
        self._buffer = []
        self.rich_log.clear()

    def _switch_topic_and_subscribe(self) -> None:
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
            self.rich_log.write(f"[red]Topic {escape(self.current_topic)} is not valid.[/]")
            return

        topic_type = type_list[0]
        try:
            msg_type = get_message(topic_type)
        except Exception as e:
            self.rich_log.write(f"[red]Failed to get message type for {escape(self.current_topic)}: "
                        f"{escape(str(e))}[/]")
            return

        self.rich_log.write(
            f"[bold]Echoing topic: [yellow]{escape(self.current_topic)}[/] "
            f"[dim]({escape(topic_type)})[/][/bold]"
        )

        qos_profile = QoSProfile(
            depth=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        try:
            self._sub = self.ros_node.create_subscription(
                msg_type, self.current_topic, self.echo_callback,
                qos_profile=qos_profile, callback_group=self.callback_group
            )

        except Exception as e:
            self.rich_log.write(f"[red]Failed to subscribe to {escape(self.current_topic)}: "
                        f"{escape(str(e))}[/]")
            self._sub = None
            return

    def echo_callback(self, msg):
        if not self._sub:
            return
        line = f"[dim]Message from {escape(self.current_topic)}: [/dim] {escape(str(msg))}"
        self._buffer.append(line)
