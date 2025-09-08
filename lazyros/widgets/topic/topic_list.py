import asyncio
from rclpy.node import Node
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container, VerticalScroll, ScrollableContainer
from textual.widgets import (
    Label,
    ListItem,
    ListView,
)
from textual.events import Key
from rich.markup import escape

from lazyros.utils.ignore_parser import IgnoreParser
import os

from rich.text import Text as RichText
from textual.events import Focus
from lazyros.utils.utility import create_css_id
from lazyros.utils.custom_widgets import CustomListView


class TopicListWidget(Container):
    """A widget to display the list of ROS topics."""

    DEFAULT_CSS = """
        TopicListWidget {
            overflow: hidden;
        }
    
        #scroll-area {
            overflow-x: auto;
            overflow-y: auto;
            height: 1fr;
        }
    """

    def __init__(self, ros_node: Node, **kwargs):
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.listview = CustomListView()

        self.ignore_parser = IgnoreParser()
        self.log(self.ignore_parser.ignore_file_path)
        
        self.topic_dict = {}
        self.selected_topic = None

        self.searching = False

    def compose(self) -> ComposeResult:
        yield self.listview

    def on_mount(self) -> None:
        self.set_interval(1, self.update_topic_list)
        if self.listview.children:
            self.listview.index = 0

    async def update_topic_list(self) -> None:
        """Fetch and update the list of topics."""

        if self.searching:
            if self.screen.focused == self.app.query_one("#footer"):
                footer = self.app.query_one("#footer")
                query = footer.search_input

                topic_list = self.apply_search_filter(query)
                visible = set(topic_list)
                hidden = set(self.topic_dict.keys()) - visible

                searching_index = len(self.topic_dict.keys()) + 1
                for n in visible:
                    css_id = create_css_id(n)
                    item = self.listview.query(f"#{css_id}").first()
                    if item:
                        item.display=True

                    index = self.listview.children.index(item) if item else None
                    if index is not None and index < searching_index:
                        searching_index = index

                self.listview.index = searching_index

                for n in hidden:
                    css_id = create_css_id(n)
                    item = self.listview.query(f"#{css_id}").first()
                    if item:
                        item.display=False

        else:
            topics = self.ros_node.get_topic_names_and_types()
            listview_topics = set(self.topic_dict.keys())

            for topic in topics:
                if self.ignore_parser.should_ignore(topic[0], 'topic'):
                    continue
                if topic[0] not in self.topic_dict:
                    self.topic_dict[topic[0]] = topic[1]
                    css_id = create_css_id(topic[0])
                    self.listview.extend([ListItem(Label(RichText.assemble(RichText(topic[0]))), id=css_id)])
                else:
                    css_id = create_css_id(topic[0])
                    match = self.listview.query(f"#{css_id}").first()
                    if match:
                        match.display = True
                    listview_topics.remove(topic[0])

            for topic in listview_topics:
                css_id = create_css_id(topic)
                match = self.listview.query(f"#{css_id}").first()
                if match:
                   match.remove() 
                self.topic_dict.pop(topic, None)

            if self.listview.index and self.listview.index >= len(self.listview.children):
                self.listview.index = max(0, len(self.listview.children) - 1)

    def on_list_view_highlighted(self, event):

        self.app.focused_pane = "left"
        self.app.current_pane_index = 1

        index = self.listview.index
        if index is None or not (0 <= index < len(self.listview.children)):
            self.selected_topic = None
            return
        item = self.listview.children[index]
        if not item.children:
            self.selected_topic = None
            return

        topic_name = str(item.children[0].renderable).strip()
        if self.selected_topic != topic_name:
            self.selected_topic = topic_name

    def apply_search_filter(self, query) -> None:
        query = query.lower().strip()
        if query:
            names = [n for n in list(self.topic_dict.keys()) if query in n.lower()]
        else:
            names = list(self.topic_dict.keys())

        return names
