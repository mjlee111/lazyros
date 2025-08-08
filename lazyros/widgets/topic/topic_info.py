import subprocess

from rclpy.node import Node
from rclpy.action import graph
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import RichLog
from rich.markup import escape

def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

class TopicInfoWidget(Container):
    """Widget for displaying ROS topic information."""

    DEFAULT_CSS = """
    InfoViewWidget {
        overflow-y: scroll;
    }
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.info_log = RichLog(wrap=True, highlight=True, markup=True, id="info-log", max_lines=1000)
        self.info_dict: dict[str, list[str]] = {}
        
        self.selected_topic = None
        self.current_topic = None

        self.topics = self.ros_node.get_topic_names_and_types()

    def compose(self) -> ComposeResult:
        yield self.info_log
        
    def on_mount(self) -> None:
        self.set_interval(0.5, self.update_info)  # Update info every 0.5 seconds

    def show_topic_info(self) -> None:
        if self.selected_topic in self.info_dict:
            return self.info_dict[self.selected_topic]
        
        topic_types = dict(self.topics).get(self.selected_topic, [])
        publisher_count = len(self.ros_node.get_publishers_info_by_topic(self.selected_topic))
        subscription_count = len(self.ros_node.get_subscriptions_info_by_topic(self.selected_topic))
        
        info_lines = []
        info_lines.append(f"Topic: {escape_markup(self.selected_topic)}")
        info_lines.append(f"  Type: {escape_markup(', '.join(topic_types))}")
        info_lines.append(f"  Publisher Count: {publisher_count}")
        info_lines.append(f"  Subscription Count: {subscription_count}")

        self.info_dict[self.selected_topic] = info_lines
        
        return info_lines 
        
    def update_info(self):
        if self.selected_topic is None:
            self.info_log.clear()
            self.info_log.write("[red]No node is selected yet.[/]")
            return

        if self.selected_topic == self.current_topic:
            return
        
        self.current_topic = self.selected_topic
        self.info_log.clear()
        info_lines = self.show_topic_info()
        self.info_log.write("\n".join(info_lines))
        