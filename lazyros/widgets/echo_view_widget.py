import subprocess
import threading
import time

from rclpy.node import Node
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import RichLog
from rich.markup import escape

def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

class EchoViewWidget(Container):
    """Widget for displaying ROS topic echo messages."""

    DEFAULT_CSS = """
    EchoViewWidget {
        overflow-y: scroll;
    }
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.echo_log = RichLog(wrap=True, highlight=True, markup=True, id="echo-log", max_lines=1000)
        self.current_topic = None
        self.echo_process = None
        self.echo_thread = None
        self.is_echoing = False
        self.update_rate = 2.0  # Display updates per second (Hz)
        self.last_update_time = 0

    def compose(self) -> ComposeResult:
        yield self.echo_log

    def start_echo(self, topic_name: str):
        """Start echoing messages from a topic."""
        if self.is_echoing:
            self.stop_echo()
        
        self.current_topic = topic_name
        self.is_echoing = True
        self.last_update_time = time.time()
        
        self.echo_log.clear()
        self.echo_log.write(f"[bold]Echoing topic: {escape_markup(topic_name)}[/bold]")
        self.echo_log.write(f"[dim]Update rate: {self.update_rate} Hz[/dim]")
        self.echo_log.write("")
        
        # Start the echo process in a separate thread
        self.echo_thread = threading.Thread(target=self._echo_topic, args=(topic_name,), daemon=True)
        self.echo_thread.start()

    def stop_echo(self):
        """Stop the current echo process."""
        if self.echo_process:
            try:
                self.echo_process.terminate()
                self.echo_process.wait(timeout=1)
            except:
                try:
                    self.echo_process.kill()
                except:
                    pass
            finally:
                self.echo_process = None
        
        self.is_echoing = False
        self.current_topic = None

    def _echo_topic(self, topic_name: str):
        """Run the ros2 topic echo command and display output."""
        try:
            command = ["ros2", "topic", "echo", topic_name]
            self.echo_process = subprocess.Popen(
                command, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            message_count = 0
            current_message = []
            
            for line in iter(self.echo_process.stdout.readline, ''):
                if not self.is_echoing:
                    break
                    
                line = line.rstrip()
                
                # Check if this is a new message separator (usually "---" or similar)
                if line == "---" or (line.startswith("---") and len(line) <= 10):
                    if current_message:
                        # Check if enough time has passed since last update
                        current_time = time.time()
                        if current_time - self.last_update_time >= (1.0 / self.update_rate):
                            message_count += 1
                            self.echo_log.write(f"[dim]Message #{message_count} [{time.strftime('%H:%M:%S')}]:[/dim]")
                            for msg_line in current_message:
                                self.echo_log.write(f"  {escape_markup(msg_line)}")
                            self.echo_log.write("")
                            self.last_update_time = current_time
                        current_message = []
                    continue
                
                # Add line to current message
                if line:
                    current_message.append(line)
            
            # Handle any remaining message
            if current_message and self.is_echoing:
                message_count += 1
                self.echo_log.write(f"[dim]Message #{message_count}:[/dim]")
                for msg_line in current_message:
                    self.echo_log.write(f"  {escape_markup(msg_line)}")
                
        except FileNotFoundError:
            self.echo_log.write("[red]ros2 command not found. Ensure ROS 2 is installed and sourced.[/]")
        except Exception as e:
            if self.is_echoing:
                self.echo_log.write(f"[red]Error echoing topic '{topic_name}': {escape_markup(str(e))}[/]")
        finally:
            if self.echo_process:
                self.echo_process = None

    def clear_log(self):
        """Clear the echo log."""
        self.echo_log.clear()
        if self.current_topic:
            self.echo_log.write(f"[bold]Echoing topic: {escape_markup(self.current_topic)}[/bold]")
            self.echo_log.write(f"[dim]Update rate: {self.update_rate} Hz[/dim]")
            self.echo_log.write("")
    
    def set_update_rate(self, rate: float):
        """Set the echo update rate in Hz."""
        self.update_rate = max(0.1, min(rate, 10.0))  # Limit between 0.1 and 10 Hz
        if self.current_topic:
            self.echo_log.write(f"[dim]Update rate changed to: {self.update_rate} Hz[/dim]")
    
    def get_update_rate(self) -> float:
        """Get the current update rate in Hz."""
        return self.update_rate