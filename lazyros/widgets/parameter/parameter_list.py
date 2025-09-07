import asyncio
import os
from typing import List, Optional
from concurrent.futures import ThreadPoolExecutor

from rclpy.node import Node
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import (
    Label,
    ListItem,
)
from rich.markup import escape

from lazyros.utils.ignore_parser import IgnoreParser
from rcl_interfaces.srv import ListParameters

from rclpy.callback_groups import ReentrantCallbackGroup
from rich.text import Text as RichText

from lazyros.utils.utility import create_css_id
from lazyros.utils.custom_widgets import CustomListView 


def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

class ParameterListWidget(Container):
    """A widget to display the list of ROS parameters."""

    DEFAULT_CSS = """
        ParameterListWidget {
            overflow: hidden;
        }

        #scroll-area {
            overflow-x: auto;
            overflow-y: auto;
            height: 1fr;
        }
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.listview = CustomListView()

        ignore_file_path = os.path.join(os.path.dirname(__file__), '../../../config/display_ignore.yaml')
        self.ignore_parser = IgnoreParser(os.path.abspath(ignore_file_path))

        self.selected_param = None
        self.node_listview = None
        self.parameter_dict = {}
        self.list_for_search = []

        self.searching = False
        self._prev_searching = False

    def compose(self) -> ComposeResult:
        yield self.listview

    def on_mount(self) -> None:
        self.set_interval(1, self.update_parameter_list)
        if self.listview.children:
            self.listview.index = 0

    def list_parameters(self, node_name):
        list_parameter_client = self.ros_node.create_client(ListParameters, f"{node_name}/list_parameters", callback_group=ReentrantCallbackGroup())
        
        req = ListParameters.Request()
        future = list_parameter_client.call_async(req)
        self.ros_node.executor.spin_until_future_complete(future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            return None
        
        result = future.result().result
        return result.names

    async def update_parameter_list(self):
        if self.searching:
            self._prev_searching = True
            if self.screen.focused == self.app.query_one("#footer"):
                footer = self.app.query_one("#footer")
                query = footer.search_input
                param_list = self.apply_search_filter(query)
                visible = set(param_list)
                hidden = set(self.list_for_search) - visible
                searching_index = len(self.list_for_search) + 1
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
            self.node_listview = self.app.query_one("#node-listview")
            node_list = list(self.node_listview.node_listview_dict.keys())
            for node in node_list:
                node_status = self.node_listview.node_listview_dict[node].status
                if node_status == "green" and node not in self.parameter_dict:
                    parameters = await asyncio.to_thread(self.list_parameters, node)
                    if not parameters:
                        return
                    self.parameter_dict[node] = []
                    for parameter in parameters:
                        label = RichText.assemble(
                            RichText(node),
                            ": ",
                            RichText(parameter)
                        )
                        should_ignore = self.ignore_parser.should_ignore(str(label), 'parameter')
                        if not should_ignore:
                            css_id = create_css_id(f"{node}-{parameter}") 
                            self.listview.extend([ListItem(Label(label), id=css_id)])
                            self.list_for_search.append(f"{node}-{parameter}")
                            self.parameter_dict[node].append(parameter)

                elif node in self.parameter_dict and node_status != "green":
                    for parameter in self.parameter_dict[node]:
                        css_id = create_css_id(f"{node}-{parameter}")
                        match = self.listview.query(f"#{css_id}")
                        if match:
                            match.remove()
                            self.list_for_search.remove(f"{node}-{parameter}")
                    self.parameter_dict.pop(node)

                elif self._prev_searching:
                    for parameter in self.parameter_dict[node]:
                        css_id = create_css_id(f"{node}-{parameter}")
                        match = self.listview.query(f"#{css_id}").first()
                        if match:
                            match.display = True

                if self.listview.index and self.listview.index >= len(self.listview.children):
                    self.listview.index = max(0, len(self.listview.children) - 1)
    
            self._prev_searching = False

    def on_list_view_highlighted(self, event):
        self.app.current_pane_index = 2
        self.app.focused_pane = "left"

        index = self.listview.index
        if index is None or not (0 <= index < len(self.listview.children)):
            self.selected_param = None
            return
        item = self.listview.children[index]
        if not item.children:
            self.selected_param = None
            return

        param_name = str(item.children[0].renderable).strip()
        if self.selected_param != param_name:
            self.selected_param = param_name

    def apply_search_filter(self, query) -> None:
        query = query.lower().strip()
        if query:
            names = [n for n in self.list_for_search if query in n.lower()]
        else:
            names = self.list_for_search
        return names
