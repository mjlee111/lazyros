from rclpy.node import Node
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import Static
from rich.markup import escape
from rich.text import Text
from rclpy.action import graph  # 使ってなければ削除OK
import asyncio
from rich.text import Text

def escape_markup(text: str) -> str:
    return escape(text)

class TopicInfoWidget(Container):
    """Widget for displaying ROS topic information."""

    DEFAULT_CSS = """
    TopicInfoWidget {           /* ← 修正: セレクタ名 */
        overflow-y: scroll;
    }
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.info_dict: dict[str, list[str]] = {}  # 文字列のリストをキャッシュ

        self.selected_topic = None
        self.current_topic = None

    def compose(self) -> ComposeResult:
        yield Static("", id="topic-info")

    def on_mount(self):
        self.set_interval(1, self.update_display)  # 1秒ごとに更新

    async def update_display(self):
        try:
            self.topic_listview = self.app.query_one("#topic-listview")
            self.selected_topic = self.topic_listview.selected_topic
        except Exception:
            self.topic_listview = None
            self.selected_topic = None

        view = self.query_one("#topic-info", Static)

        if self.selected_topic is None:
            # ← 素の文字列ではなく Text.from_markup を使う
            view.update(Text.from_markup("[red]No topic is selected yet.[/]"))
            return

        if self.selected_topic == self.current_topic:
            return

        self.current_topic = self.selected_topic

        info_lines = await asyncio.to_thread(self.show_topic_info)

        if info_lines:
            view.update(Text.from_markup("\n".join(info_lines)))

    def show_topic_info(self) -> list[str] | None:
        if self.selected_topic in self.info_dict:
            return self.info_dict[self.selected_topic]

        topic_dict = self.topic_listview.topic_dict if self.topic_listview else None
        if not topic_dict:
            return [f"[red]Topic {escape_markup(self.selected_topic)} is not set.[/]"]

        topic_types = dict(topic_dict).get(self.selected_topic, [])

        publisher_count = len(self.ros_node.get_publishers_info_by_topic(self.selected_topic))
        subscription_count = len(self.ros_node.get_subscriptions_info_by_topic(self.selected_topic))

        info_lines: list[str] = []
        info_lines.append(f"Topic: {escape_markup(self.selected_topic)}")
        info_lines.append(f"  Type: {escape_markup(', '.join(topic_types))}")
        info_lines.append(f"  Publisher Count: {publisher_count}")
        info_lines.append(f"  Subscription Count: {subscription_count}")

        self.info_dict[self.selected_topic] = info_lines
        return info_lines
