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
    ListView,
)
from rich.markup import escape

from lazyros.utils.ignore_parser import IgnoreParser
from rcl_interfaces.srv import ListParameters

from rclpy.callback_groups import ReentrantCallbackGroup
from rich.text import Text as RichText
from textual.events import Focus

def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)


class MyListView(ListView):
    """Custom ListView that automatically focuses on mount."""

    def on_focus(self, event: Focus) -> None:
        if self.children and not self.index:
            self.index = 0


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
        self.listview = MyListView()

        ignore_file_path = os.path.join(os.path.dirname(__file__), '../../../config/display_ignore.yaml')
        self.ignore_parser = IgnoreParser(os.path.abspath(ignore_file_path))

        self.selected_param = None
        self.node_listview = None
        self.parameter_dict = {}
        self.list_for_search = []

        self.searching = False

    def compose(self) -> ComposeResult:
        yield self.listview

    def on_mount(self) -> None:
        self.set_interval(0.1, self.update_parameter_list)
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
        if not self.listview.index and not self.searching:
            self.listview.index = 0

        if self.searching:
            if self.screen.focused == self.app.query_one("#footer"):
                self.listview.clear()
                footer = self.app.query_one("#footer")
                query = footer.input
                param_list = self.apply_search_filter(query)
                self.listview.extend(param_list)

        else:
            self.node_listview = self.app.query_one("#node-listview")

            node_list = list(self.node_listview.node_listview_dict.keys())
            for node in node_list:
                if node not in self.parameter_dict:
                    parameters = self.list_parameters(node)
                    if not parameters:
                        return
                    
                    self.parameter_dict[node] = parameters
                    for parameter in parameters:

                        label = RichText.assemble(
                            RichText(node),
                            ": ",
                            RichText(parameter)
                        )
                        should_ignore = self.ignore_parser.should_ignore(str(label), 'parameter')
                        if not should_ignore:
                            css_id = f"{node}-{parameter}".lstrip("/").replace("/", "-")
                            self.listview.extend([ListItem(Label(label), id=css_id)])

                #elif self.node_listview.node_listview_dict[node].status != "green":
                #    for parameter in self.parameter_dict[node]:
                #        css_id = f"{node}-{parameter}".lstrip("/").replace("/", "-")
                #        match = self.listview.query(f"#{css_id}")
                #        if match:
                #            match.remove()
#
                #    self.parameter_dict.pop(node)

                if self.listview.index and self.listview.index >= len(self.listview.children):
                    self.listview.index = max(0, len(self.listview.children) - 1)
        
                self.listview.refresh(layout=True)

            #if len(self.listview.children) != len(self.list_for_search):
            #    need_update = True
#
            #if not need_update:
            #    return
#
            ## update parameter listview
            #self.listview.clear()
            #parameter_list = []
            #self.list_for_search = []
            #node_list = list(self.parameter_dict.keys())
            #for node in node_list:
            #    for parameter in self.parameter_dict[node]:
            #        label = RichText.assemble(
            #            RichText(node),
            #            ": ",
            #            RichText(parameter)
            #        )
            #        should_ignore = self.ignore_parser.should_ignore(str(label), 'parameter')
            #        if not should_ignore:
            #            parameter_list.append(ListItem(Label(label)))
            #            self.list_for_search.append(f"{node}: {parameter}")
#
            #self.listview.extend(parameter_list)

    def on_list_view_highlighted(self, event):

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

        filtered_parameters = []
        for n in names:
            filtered_parameters.append(ListItem(Label(RichText.assemble(RichText(n)))))
        return filtered_parameters
