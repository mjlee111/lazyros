import subprocess
import threading
import re # For parsing node and param name
from typing import List, Optional, Tuple

from textual.app import ComposeResult, App
from textual.binding import Binding
from textual.containers import Container, VerticalScroll, Horizontal, Center
from textual.css.query import DOMQuery
from textual.reactive import reactive
from textual.screen import Screen, ModalScreen 
from textual.widgets import (
    Label,
    Static,
    Button,
    Input,
)
from rich.markup import escape as escape_markup


class SetParameterModal(ModalScreen[None]):
    """A modal screen to set a parameter's value."""

    CSS = """
    SetParameterModal {
        align: center middle; /* Center the modal itself on screen */
        layer: modal;
    }

    #set-param-dialog {
        width: 40%;
        height: 30%;
        border: round white;
        background: $background;
        padding: 1;
        layout: vertical; /* Use vertical layout for children */
    }

    #set-param-title {
        dock: top;
        width: 100%;
        text-align: center;
        padding: 1;
        background: $primary-background-darken-1;
    }

    #set-param-type {
        text-align: left;
        padding: 1;
        width: 1fr; /* Assign fractional width */
    }

    #set-param-input {
        margin-bottom: 1;
        width: 2fr; /* Assign fractional width */
    }

    #set-param-button {
        align: center middle; /* Corrected align property with two values */
        width: 1fr; /* Assign fractional width */
    }

    #set-param-status {
        width: 100%;
        text-align: center;
        padding: 1;
    }
    
    #set-param-horizontal-container { /* CSS for the new horizontal container */
        width: 100%; /* Ensure the container takes full width */
        margin-top: 1; /* Add padding for spacing */
    }
    
    #set-param-instruction {
        dock: bottom;
        width: 100%;
        text-align: center;
        padding: 1;
    }
    """

    BINDINGS = [
        Binding("escape, q", "dismiss", "Close", show=True),
    ]

    def __init__(self, node_name: str, param_name: str, param_type: str, **kwargs) -> None:
        super().__init__(**kwargs)
        self.node_name = node_name
        self.param_name = param_name
        self.param_type = param_type

    def compose(self) -> ComposeResult:
        yield Container(
            Static(f"Set Parameter: {self.node_name}/{self.param_name}", id="set-param-title"),
            Horizontal(
                Static(f"Type: {self.param_type}", id="set-param-type"),
                Input(placeholder=f"Enter value ({self.param_type})", id="set-param-input"),
                Button("Set", variant="primary", id="set-param-button"), 
                id="set-param-horizontal-container" 
            ),
            Static("", id="set-param-status"),
            Label("Press 'q' to quit.", id="set-param-instruction"),
            id="set-param-dialog",
        )
    
    def action_dismiss(self) -> None:
        """Action to dismiss the modal."""
        self.dismiss()

    def on_mount(self) -> None:
        """Called when the modal is mounted."""
        self.query_one("#set-param-dialog").focus()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "set-param-button":
            new_value = self.query_one("#set-param-input", Input).value
            status_label = self.query_one("#set-param-status", Static)
            status_label.update("Setting parameter...")

            try:
                cmd = f"ros2 param set {self.node_name} {self.param_name} {new_value}"
                process_result = subprocess.run(
                    cmd, shell=True, capture_output=True, text=True, timeout=5
                )

                if process_result.returncode == 0 and "successful" in process_result.stdout:
                    status_label.update(f"[green]Parameter '{self.param_name}' set to '{new_value}' successfully![/green]")
                else:
                    err_msg = process_result.stderr.strip() if process_result.stderr else "Unknown error"
                    status_label.update(f"[red]Error setting parameter: {escape_markup(err_msg)}[/red]")

            except subprocess.TimeoutExpired:
                status_label.update("[red]Error setting parameter: Timeout expired.[/red]")
            except Exception as e:
                status_label.update(f"[red]An error occurred setting parameter: {escape_markup(str(e))}[/red]")
