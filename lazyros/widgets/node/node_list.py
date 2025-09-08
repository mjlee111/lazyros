import os
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set

from rclpy.node import Node
from rich.markup import escape
from rich.text import Text as RichText
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container
from textual.events import Focus, Key
from textual.widgets import Label, ListItem, ListView

from lazyros.utils.custom_widgets import CustomListView
from lazyros.utils.ignore_parser import IgnoreParser
from lazyros.utils.utility import create_css_id

            
@dataclass
class NodeData:
    """Data class to store information about a ROS node.
    
    Attributes:
        full_name: The full name of the node including namespace
        status: The current status of the node (e.g., "green", "red")
        index: The index position in the list view
        namespace: The namespace of the node
        node_name: The name of the node without namespace
    """
    full_name: str
    status: str
    index: int
    namespace: str = ""
    node_name: str = ""

class NodeListWidget(Container):
    def __init__(self, ros_node: Node, **kwargs: Any) -> None:
        """Initialize the NodeListWidget.
        
        Args:
            ros_node: The ROS node instance for communication
            **kwargs: Additional keyword arguments passed to the parent Container
        """
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.listview = CustomListView()
        self.node_listview_dict = {}
        self.searching = False
        
        self.selected_node_name = None
        self.ignore_parser = IgnoreParser()

    def compose(self) -> ComposeResult:
        """Compose the widget layout.
        
        Returns:
            ComposeResult: A generator yielding widget components
        """
        yield self.listview

    def on_mount(self) -> None:
        """Handle the widget mount event.
        
        Sets up periodic interval to update node list and initializes list view index.
        """
        self.set_interval(1, self.update_node_list)

        if self.listview.children:
            self.listview.index = 0

    async def update_node_list(self) -> None:
        """Update the list of nodes.
        
        Fetches current nodes from ROS graph and updates the display.
        Handles search filtering if active, otherwise shows all available nodes.
        """
        """Update the list of nodes."""
        try:
            if not self.listview.index and not self.searching:
                self.listview.index = 0

            if self.searching:
                try:
                    if self.screen.focused == self.app.query_one("#footer"):
                        footer = self.app.query_one("#footer")
                        query = footer.search_input

                        node_list = self.apply_search_filter(query)
                        visible = set(node_list)
                        hidden = set(self.node_listview_dict.keys()) - visible

                        searching_index = len(self.node_listview_dict.keys()) + 1
                        for n in visible:
                            try:
                                index = self.node_listview_dict[n].index
                                if index < len(self.listview.children):
                                    item = self.listview.children[index]
                                    item.display=True

                                    if index < searching_index:
                                        searching_index = index
                            except (IndexError, KeyError):
                                continue

                        if searching_index < len(self.listview.children):
                            self.listview.index = searching_index

                        for n in hidden:
                            try:
                                index = self.node_listview_dict[n].index
                                if index < len(self.listview.children):
                                    item = self.listview.children[index]
                                    item.display=False
                            except (IndexError, KeyError):
                                continue
                except Exception:
                    pass
            else:
                try:
                    nodes_and_namespaces = self.ros_node.get_node_names_and_namespaces()
                    launched_node_set =list(self.node_listview_dict.keys())
                except Exception:
                    return

                for tuple in nodes_and_namespaces:
                    try:
                        node = tuple[0]
                        namespace = tuple[1]
                        if namespace == "/":
                            node_name = namespace + node 
                        else:
                            node_name = namespace + "/" + node

                        if self.ignore_parser.should_ignore(node_name, 'node'):
                            continue

                        if node_name not in launched_node_set:
                            try:
                                index = len(self.listview.children)
                                self.listview.extend([ListItem(Label(RichText.assemble(RichText("●", style="bold green"), "    ", RichText(node_name))))])
                                self.node_listview_dict[node_name] = NodeData(full_name=node_name, status="green", index=index, namespace=namespace, node_name=node)
                            except Exception:
                                continue
                        else:
                            try:
                                index = self.node_listview_dict[node_name].index
                                if index < len(self.listview.children) and not self.listview.children[index].display:
                                    self.listview.children[index].display = True

                                if self.node_listview_dict[node_name].status != "green":
                                    self.node_listview_dict[node_name].status = "green"
                                    index = self.node_listview_dict[node_name].index
                                    if index < len(self.listview.children):
                                        item = self.listview.children[index]
                                        label = item.query_one(Label)
                                        label.update(RichText.assemble(RichText("●", style="bold green"), "    ", RichText(node_name)))
                                launched_node_set.remove(node_name)
                            except (IndexError, KeyError):
                                continue
                    except Exception:
                        continue

                # Set nodes that are no longer launched to red
                for dead_node in launched_node_set:
                    try:
                        if self.node_listview_dict[dead_node].status == "green":
                            self.node_listview_dict[dead_node].status = "red"
                            index = self.node_listview_dict[dead_node].index
                            if index < len(self.listview.children):
                                item = self.listview.children[index]
                                label = item.query_one(Label)
                                label.update(RichText.assemble(RichText("●", style="red"), "    ", RichText(dead_node)))
                    except (IndexError, KeyError):
                        continue
        except Exception:
            pass

    def on_list_view_highlighted(self, event: Any) -> None:
        """Handle list view highlighting events.
        
        Args:
            event: The list view highlight event
        """
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

    def apply_search_filter(self, query: str) -> List[str]:
        """Apply search filter to node list.
        
        Args:
            query: The search query string
            
        Returns:
            List[str]: List of node names matching the query
        """
        query = query.lower().strip()
        if query:
            names = [n for n in list(self.node_listview_dict.keys()) if query in n.lower()]
        else:
            names = list(self.node_listview_dict.keys())
        return names
