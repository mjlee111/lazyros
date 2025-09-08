import asyncio
from typing import Any, Dict, List, Optional

from rclpy.node import Node
from rich.markup import escape
from rich.text import Text
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import Static


class TopicInfoWidget(Container):
    """Widget for displaying ROS topic information."""

    DEFAULT_CSS = """
    TopicInfoWidget {
        overflow-y: scroll;
    }
    """

    def __init__(self, ros_node: Node, **kwargs: Any) -> None:
        """Initialize the TopicInfoWidget.
        
        Args:
            ros_node: The ROS node instance for communication
            **kwargs: Additional keyword arguments passed to the parent Container
        """
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.info_dict: dict[str, list[str]] = {}

        self.selected_topic = None
        self.current_topic = None

    def compose(self) -> ComposeResult:
        """Compose the widget layout.
        
        Returns:
            ComposeResult: A generator yielding widget components
        """
        yield Static("", id="topic-info")

    def on_mount(self) -> None:
        """Handle the widget mount event.
        
        Sets up periodic interval to update display.
        """
        self.set_interval(1, self.update_display)

    async def update_display(self) -> None:
        """Update the topic information display.
        
        Shows topic details for the currently selected topic.
        Updates are skipped if no topic is selected.
        """
        try:
            self.topic_listview = self.app.query_one("#topic-listview")
            self.selected_topic = self.topic_listview.selected_topic
        except Exception:
            self.topic_listview = None
            self.selected_topic = None

        view = self.query_one("#topic-info", Static)

        if self.selected_topic is None:
            view.update(Text.from_markup("[red]No topic is selected yet.[/]"))
            return

        if self.selected_topic == self.current_topic:
            return

        self.current_topic = self.selected_topic

        info_lines = await asyncio.to_thread(self.show_topic_info)

        if info_lines:
            view.update(Text.from_markup("\n".join(info_lines)))

    def show_topic_info(self) -> Optional[List[str]]:
        """Retrieve and format topic information.
        
        Returns:
            Optional[List[str]]: A list of formatted strings containing topic information,
                                or None if information could not be retrieved
        """
        if self.selected_topic in self.info_dict:
            return self.info_dict[self.selected_topic]

        topic_dict = self.topic_listview.topic_dict if self.topic_listview else None
        if not topic_dict:
            return [f"[red]Topic {escape(self.selected_topic)} is not set.[/]"]

        topic_types = dict(topic_dict).get(self.selected_topic, [])

        publisher_count = len(self.ros_node.get_publishers_info_by_topic(self.selected_topic))
        subscription_count = len(self.ros_node.get_subscriptions_info_by_topic(self.selected_topic))

        info_lines: list[str] = []

        info_lines = [
            f"[bold cyan]Topic:[/] [yellow]{escape(self.selected_topic)}[/]",
            f"[bold cyan]Type:[/] [green]{escape(', '.join(topic_types))}[/]",
            f"[bold cyan]Publishers:[/] [magenta]{publisher_count}[/]",
            f"[bold cyan]Subscriptions:[/] [magenta]{subscription_count}[/]",
        ]

        self.info_dict[self.selected_topic] = info_lines
        return info_lines
