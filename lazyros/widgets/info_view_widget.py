import subprocess

from rclpy.node import Node
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import RichLog
from rich.markup import escape

def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

class InfoViewWidget(Container):
    """Widget for displaying ROS node information."""

    DEFAULT_CSS = """
    InfoViewWidget {
        overflow-y: scroll;
    }
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node # May not be directly used if info comes from subprocess
        self.info_log = RichLog(wrap=True, highlight=True, markup=True, id="info-log", max_lines=1000)
        self.info_dict: dict[str, list[str]] = {} # Cache for node info

    def compose(self) -> ComposeResult:
        yield self.info_log

    def update_info(self, node_name: str):
        """Update the displayed node information using `ros2 node info` output.
        node_name is expected to be the name without a leading slash, e.g., 'talker' or 'namespace/nodename'.
        """
        
        # Check if we have cached info, display it immediately, but also refresh in background
        if node_name in self.info_dict:
            self._display_cached_info(node_name)
            return
        
        print(f"InfoViewWidget.update_info: Fetching info for node: {node_name}")
        
        try:
            # `ros2 node info` expects the node name, not the full path
            # If NodeListWidget passes "talker", command is "ros2 node info talker"
            # If NodeListWidget passes "ns1/talker", command is "ros2 node info ns1/talker"
            command = ["ros2", "node", "info", node_name] 
            result = subprocess.run(command, capture_output=True, text=True, check=True, timeout=5) # Added timeout

            lines = result.stdout.splitlines()
            sections: dict[str, list[str]] = {
                "Subscribers": [], "Publishers": [], "Service Servers": [],
                "Service Clients": [], "Action Servers": [], "Action Clients": []
            }
            current_section = None

            for line_raw in lines:
                line = line_raw.strip() # Process stripped line
                # Check for section headers
                if line.endswith("Subscribers:"): current_section = "Subscribers"
                elif line.endswith("Publishers:"): current_section = "Publishers"
                elif line.endswith("Service Servers:"): current_section = "Service Servers"
                elif line.endswith("Service Clients:"): current_section = "Service Clients"
                elif line.endswith("Action Servers:"): current_section = "Action Servers"
                elif line.endswith("Action Clients:"): current_section = "Action Clients"
                
                if current_section: # Add line to current section if one is active
                    # Add the original (potentially indented) line to preserve formatting for section items
                    sections[current_section].append(line_raw) # Use line_raw to keep indentation

            formatted_lines = []
            for section_name, item_lines in sections.items():
                if not item_lines: continue # Skip empty sections

                # The first line in item_lines is the section title itself, which might be indented
                # We want the title to be bold, and items to be clickable
                title = item_lines[0].strip() # Get the clean title
                formatted_lines.append(f"[bold]{escape_markup(title)}[/bold]")

                for item_line_raw in item_lines[1:]: # Process items under the title
                    item_line = item_line_raw.strip()
                    if not item_line.startswith("/"): continue # Skip non-path items

                    # For Subscribers and Publishers, make topic and type clickable
                    if section_name in ["Subscribers", "Publishers"]:
                        parts = item_line.split(":", 1)
                        topic_path = parts[0].strip()
                        msg_type = parts[1].strip() if len(parts) > 1 else ""
                        
                        # Ensure topic_path and msg_type are properly escaped for @click
                        topic_arg = topic_path.replace("'", "\\'")
                        msg_type_arg = msg_type.replace("'", "\\'")

                        formatted_lines.append(
                            f"  [@click=app.handle_topic_click('{topic_arg}')]{escape_markup(topic_path)}[/]: "
                            f"[@click=app.handle_message_click('{msg_type_arg}')]{escape_markup(msg_type)}[/]"
                        )
                    else: # For other sections, just display the item
                        formatted_lines.append(f"  {escape_markup(item_line)}")
                formatted_lines.append("") # Add a blank line after each section

            self.info_log.clear()
            for fl_line in formatted_lines:
                self.info_log.write(fl_line)
            self.info_dict[node_name] = formatted_lines
            #self.info_log.scroll_home() # Scroll to top

        except FileNotFoundError:
            self.info_log.clear()
            self.info_log.write("[red]ros2 command not found. Ensure ROS 2 is installed and sourced.[/]")
        except subprocess.TimeoutExpired:
            self.info_log.clear()
            self.info_log.write(f"[red]Timeout fetching info for node: {node_name}[/]")
        except subprocess.CalledProcessError as e:
            self.info_log.clear()
            self.info_log.write(f"[red]Error fetching info for node '{node_name}':[/]\n{escape_markup(e.stderr or e.stdout)}")
        except Exception as e:
            self.info_log.clear()
            self.info_log.write(f"[red]Unexpected error fetching info for '{node_name}': {escape_markup(str(e))}[/]")
    
    def _display_cached_info(self, node_name: str):
        """Display cached information for a node."""
        if node_name in self.info_dict:
            self.info_log.clear()
            for line in self.info_dict[node_name]:
                self.info_log.write(line)
