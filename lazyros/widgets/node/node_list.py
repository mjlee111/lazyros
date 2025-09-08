import os
from dataclasses import dataclass
from rclpy.node import Node
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container
from textual.widgets import Label, ListItem, ListView
from textual.events import Focus, Key
from rich.markup import escape
from rich.text import Text as RichText
from lazyros.utils.ignore_parser import IgnoreParser
from lazyros.utils.utility import create_css_id
from lazyros.utils.custom_widgets import CustomListView

            
@dataclass
class NodeData:
    full_name: str
    status: str
    index: int
    namespace: str = ""
    node_name: str = ""

class NodeListWidget(Container):
    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.listview = CustomListView()
        self.node_listview_dict = {}
        self.searching = False
        
        self.selected_node_name = None
        self.ignore_parser = IgnoreParser()

    def compose(self) -> ComposeResult:
        yield self.listview

    def on_mount(self) -> None:
        self.set_interval(1, self.update_node_list)

        if self.listview.children:
            self.listview.index = 0

    async def update_node_list(self) -> None:
        """Update the list of nodes."""

        if not self.listview.index and not self.searching:
            self.listview.index = 0

        if self.searching:
            if self.screen.focused == self.app.query_one("#footer"):
                footer = self.app.query_one("#footer")
                query = footer.search_input

                node_list = self.apply_search_filter(query)
                visible = set(node_list)
                hidden = set(self.node_listview_dict.keys()) - visible

                searching_index = len(self.node_listview_dict.keys()) + 1
                for n in visible:
                    index = self.node_listview_dict[n].index
                    item = self.listview.children[index]
                    item.display=True

                    if index < searching_index:
                        searching_index = index

                self.listview.index = searching_index

                for n in hidden:
                    index = self.node_listview_dict[n].index
                    item = self.listview.children[index]
                    item.display=False
        else:
            nodes_and_namespaces = self.ros_node.get_node_names_and_namespaces()
            launched_node_set =list(self.node_listview_dict.keys())

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
                        index = len(self.listview.children)
                        self.listview.extend([ListItem(Label(RichText.assemble(RichText("●", style="bold green"), "    ", RichText(node_name))))])
                        self.node_listview_dict[node_name] = NodeData(full_name=node_name, status="green", index=index, namespace=namespace, node_name=node)
                else:
                    index = self.node_listview_dict[node_name].index
                    if not self.listview.children[index].display:
                        self.listview.children[index].display = True

                    if self.node_listview_dict[node_name].status != "green":
                        self.node_listview_dict[node_name].status = "green"
                        index = self.node_listview_dict[node_name].index
                        item = self.listview.children[index]
                        label = item.query_one(Label)
                        label.update(RichText.assemble(RichText("●", style="bold green"), "    ", RichText(node_name)))
                    launched_node_set.remove(node_name)

            # Set nodes that are no longer launched to red
            for dead_node in launched_node_set:
                if self.node_listview_dict[dead_node].status == "green":
                    self.node_listview_dict[dead_node].status = "red"
                    index = self.node_listview_dict[dead_node].index
                    item = self.listview.children[index]
                    label = item.query_one(Label)
                    label.update(RichText.assemble(RichText("●", style="red"), "    ", RichText(dead_node)))

    def on_list_view_highlighted(self, event):
        self.app.focused_pane = "left"
        self.app.current_pane_index = 0

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
        return names
