import subprocess
import threading
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container
from textual.widgets import Label, Log
from textual.screen import ModalScreen
from rich.markup import escape as escape_markup


class TopicEchoModal(ModalScreen[None]):
    """A modal screen to display 'ros2 topic echo' output."""

    CSS = """
    TopicEchoModal {
        align: center middle;
        layer: modal;
    }

    #echo-modal-container {
        width: 40%; 
        height: 30%; 
        border: round white;
        background: $background;
    }

    #echo-modal-title {
        dock: top;
        width: 100%;
        text-align: center;
        padding: 1;
        background: $primary-background-darken-1; 
    }

    #echo-log {
        width: 100%;
        height: 1fr;
        border: round $primary;
        margin: 0 1;
    }

    #echo-modal-instruction {
        dock: bottom;
        width: 100%;
        text-align: center;
        padding: 1;
    }
    """

    BINDINGS = [
        Binding("q", "dismiss", "Close & Stop Echo"),
    ]

    def __init__(self, topic_name: str, topic_type: str, **kwargs) -> None:
        super().__init__(**kwargs)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.echo_process: subprocess.Popen | None = None
        self.log_widget: Log | None = None
        self._stop_event = threading.Event() 

    def compose(self) -> ComposeResult:
        """Compose the modal dialog."""
        self.log_widget = Log(id="echo-log", highlight=True)
        yield Container(
            Label(f"Echoing Topic: {escape_markup(self.topic_name)} ({escape_markup(self.topic_type)})", id="echo-modal-title"),
            self.log_widget,
            Label("Press 'q' to close and stop echoing.", id="echo-modal-instruction"),
            id="echo-modal-container",
        )

    def on_mount(self) -> None:
        """Start echoing when the modal is mounted."""
        self.start_echo()

    def start_echo(self) -> None:
        """Starts the 'ros2 topic echo' subprocess and streams its output."""
        if self.echo_process is not None or not self.log_widget:
            return

        if not self.topic_type:
            self.log_widget.write_line("[bold red]Error: Topic type is missing. Cannot echo.[/]")
            return

        try:
            command = ["ros2", "topic", "echo", self.topic_name, self.topic_type]
            self.log_widget.write_line(f"Starting: {' '.join(map(escape_markup, command))}")
            
            self.echo_process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True, 
                bufsize=1,  
            )

            stdout_thread = threading.Thread(target=self._read_stream, args=(self.echo_process.stdout, "STDOUT"))
            stdout_thread.daemon = True 
            stdout_thread.start()

            stderr_thread = threading.Thread(target=self._read_stream, args=(self.echo_process.stderr, "STDERR"))
            stderr_thread.daemon = True
            stderr_thread.start()

        except FileNotFoundError:
            if self.log_widget:
                self.log_widget.write_line(f"[bold red]Error: 'ros2' command not found. Is ROS2 installed and sourced?[/]")
            self.echo_process = None
        except Exception as e:
            if self.log_widget:
                self.log_widget.write_line(f"[bold red]Error starting echo process: {escape_markup(str(e))}[/]")
            self.echo_process = None

    def _read_stream(self, stream, stream_name: str) -> None:
        """Reads lines from the stream and posts them to the Log widget."""
        if not self.log_widget:
            return
            
        prefix = ""
        if stream_name == "STDERR":
            prefix = "[bold red]STDERR: [/]"

        try:
            while not self._stop_event.is_set():
                if stream is None: break # Stream might be None if Popen failed partially
                line = stream.readline()
                if not line: 
                    if self.echo_process and self.echo_process.poll() is not None: 
                        break 
                    if self._stop_event.is_set(): # Check again in case event was set while readline blocked
                        break
                    continue 
                
                self.app.call_from_thread(self.log_widget.write_line, prefix + escape_markup(line.rstrip()))
        except Exception:
            # Errors during reading are logged if possible, but thread should exit cleanly.
            # self.app.call_from_thread(self.log_widget.write_line, f"[bold red]Error reading {stream_name}: {escape_markup(str(e))}[/]")
            pass # Avoid complex error handling in thread that might itself fail
        finally:
            try:
                if stream:
                    stream.close()
            except Exception:
                pass

    def stop_echo(self) -> None:
        """Stops the 'ros2 topic echo' subprocess."""
        self._stop_event.set() 
        process_to_stop = self.echo_process
        self.echo_process = None # Prevent further operations on this process

        if process_to_stop and process_to_stop.poll() is None: 
            if self.log_widget:
                # Use call_from_thread if stop_echo can be called from non-main thread
                # For action_dismiss and on_remove, it's main thread.
                self.log_widget.write_line("[yellow]Stopping echo process...[/]")
            try:
                process_to_stop.terminate() 
                try:
                    process_to_stop.wait(timeout=1) 
                except subprocess.TimeoutExpired:
                    if self.log_widget:
                        self.log_widget.write_line("[orange3]Echo process did not terminate gracefully, killing...[/]")
                    process_to_stop.kill() 
                    process_to_stop.wait(timeout=1) 
            except Exception as e:
                if self.log_widget:
                    self.log_widget.write_line(f"[bold red]Error stopping echo process: {escape_markup(str(e))}[/]")
            
        if self.log_widget and self._stop_event.is_set(): # Check if already stopped
             # Ensure this message is only logged once or if actually stopped now.
             # This might be tricky if called multiple times.
             pass # Message might be redundant if threads log completion.

    def action_dismiss(self, result: None = None) -> None:
        """Called when the modal is dismissed."""
        self.stop_echo()
        self.dismiss(result)

    async def on_remove(self) -> None:
        """Ensure echo is stopped when the screen is removed."""
        self.stop_echo()
