import subprocess
import asyncio
from rclpy.node import Node
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container
from textual.widgets import Label, ListItem, ListView
from rich.markup import escape
from rich.text import Text as RichText
from dataclasses import dataclass

from lazyros.utils.ignore_parser import IgnoreParser
from lazyros.modals.lifecycle_modal import LifecycleModal

import os
import signal

from textual.events import Focus
import re

def escape_markup(text: str) -> str:
    return escape(text)


class MyListView(ListView):
    """Custom ListView that automatically focuses on mount."""

    def on_focus(self, event: Focus) -> None:
        if self.children and not self.index:
            self.index = 0

@dataclass
class NodeData:
    full_name: str
    status: str
    namespace: str = ""
    node_name: str = ""

class NodeListWidget(Container):
    def __init__(self, ros_node: Node, ignore_file_path='config/display_ignore.yaml', **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.listview = MyListView()
        self.node_listview_dict = {}
        self.searching = False
        
        self.selected_node_name = None
        ignore_file_path = os.path.join(os.path.dirname(__file__), '../../../config/display_ignore.yaml')
        self.ignore_parser = IgnoreParser(os.path.abspath(ignore_file_path))

    def compose(self) -> ComposeResult:
        yield self.listview

    def on_mount(self) -> None:
        asyncio.create_task(self.update_node_list())
        self.set_interval(0.1, lambda: asyncio.create_task(self.update_node_list()))

        if self.listview.children:
            self.listview.index = 0

    async def update_node_list(self) -> None:
        """Update the list of nodes."""

        if not self.listview.index and not self.searching:
            self.listview.index = 0

        if self.searching:
            if self.screen.focused == self.app.query_one("#footer"):
                self.listview.clear()
                footer = self.app.query_one("#footer")
                query = footer.input
                node_list = self.apply_search_filter(query)
                self.listview.extend(node_list)

        else:
            self.log(f"{self.node_listview_dict.keys()}")
            nodes_and_namespaces = self.ros_node.get_node_names_and_namespaces()
            launched_node_set =list(self.node_listview_dict.keys())
            need_update = False

            for tuple in nodes_and_namespaces:
                node = tuple[0]
                namespace = tuple[1]
                if namespace == "/":
                    node_name = namespace + node 
                else:
                    node_name = namespace + "/" + node

                if self.ignore_parser.should_ignore(node_name, 'node'):
                    continue

                if node_name not in launched_node_set:
                    need_update = True
                    # If the node is not in the launched nodes, add it
                    self.node_listview_dict[node_name] = NodeData(full_name=node_name, status="green", namespace=namespace, node_name=node)
                else:
                    if self.node_listview_dict[node_name].status != "green":
                        need_update = True
                        self.node_listview_dict[node_name].status = "green"
                    launched_node_set.remove(node_name)
           
            # Set nodes that are no longer launched to red
            for dead_node in launched_node_set:
                if self.node_listview_dict[dead_node].status == "green":
                    self.node_listview_dict[dead_node].status = "red"
                    self.log(f"here3")
                    need_update = True
    
            if len(self.listview.children) != len(self.node_listview_dict.keys()):
                need_update = True

            if not need_update:
                return
                    
            nodes_list = list(self.node_listview_dict.keys())
            sorted_nodes = sorted(nodes_list)
              
            # update node listview 
            self.listview.clear()
            nodes = []
            for node in sorted_nodes:
                status = self.node_listview_dict[node].status
                label = RichText.assemble(
                   RichText("●", style=f"bold {status}"),
                   "    ",
                   RichText(node)
                )
                nodes.append(ListItem(Label(label)))
                 
            self.listview.extend(nodes)

                #current_index = self.listview.index
                #if current_index is not None and current_index < len(nodes):
                #    self.listview.index = current_index
                #elif nodes:
                #    self.listview.index = 0

    def on_list_view_highlighted(self, event):
        index = self.listview.index
        if index is None or not (0 <= index < len(self.listview.children)):
            self.selected_node_name = None
            return
        item = self.listview.children[index]
        if not item.children:
            self.selected_node_name = None
            return

        # Extract the display name
        name_str = str(item.children[0].renderable).strip()
        # Extract node name after the first slash (if any)
        node_name = name_str.split("/", 1)[-1] if "/" in name_str else name_str

        if self.selected_node_name != node_name:
            self.selected_node_name = node_name

    def apply_search_filter(self, query) -> None:
        query = query.lower().strip()
        if query:
            names = [n for n in list(self.node_listview_dict.keys()) if query in n.lower()]
        else:
            names = list(self.node_listview_dict.keys())

        filtered_nodes = []
        for n in names:
            status = self.node_listview_dict[n].status
            label = RichText.assemble(
                RichText("●", style=f"bold {status}"),
                "    ",
                RichText(n)
            )
            filtered_nodes.append(ListItem(Label(label)))
        return filtered_nodes
