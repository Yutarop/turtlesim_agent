import asyncio
import sys
import tkinter as tk
from asyncio import run_coroutine_threadsafe
from tkinter import scrolledtext

from turtlesim_agent.interface.gui_interface import GUIAgentInterface


class ChatUI:
    def __init__(
        self, root, interface: GUIAgentInterface, loop: asyncio.AbstractEventLoop
    ):
        self.interface = interface
        self.root = root
        self.loop = loop
        self.root.title("TurtleBot Agent Chat")
        self.root.configure(bg="black")

        self.chat_display = scrolledtext.ScrolledText(
            root,
            wrap=tk.WORD,
            height=20,
            width=60,
            state="disabled",
            bg="black",
            fg="white",
            insertbackground="white",
            font=("Consolas", 12),
        )
        self.chat_display.pack(padx=10, pady=(10, 0))

        self.entry_frame = tk.Frame(root, bg="black")
        self.entry_frame.pack(fill=tk.X, padx=10, pady=(10, 10))

        self.entry = tk.Entry(
            self.entry_frame,
            width=50,
            bg="black",
            fg="white",
            insertbackground="white",
            font=("Consolas", 12),
        )
        self.entry.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 5))
        self.entry.bind("<Return>", self.send_message)

        self.send_button = tk.Button(
            self.entry_frame,
            text="Send",
            command=self.send_message,
            bg="skyblue",
            fg="black",
            font=("Consolas", 11, "bold"),
        )
        self.send_button.pack(side=tk.RIGHT)

        self.root.protocol("WM_DELETE_WINDOW", self.quit_app)

    def send_message(self, event=None):
        user_input = self.entry.get().strip()
        if not user_input:
            return

        if user_input.lower() in ["quit", "exit"]:
            self.quit_app()

        self._append_chat("User", user_input, "white", "white")
        self.entry.delete(0, tk.END)

        # threading.Thread(target=self.process_agent_response, args=(user_input,)).start()
        run_coroutine_threadsafe(self.process_agent_response(user_input), self.loop)

    async def process_agent_response(self, user_input):
        response = await self.interface.send_user_input(user_input)
        self._append_chat("TurtleBot Agent", response, "#00FF00", "#00FF00")

    def _append_chat(self, speaker, message, label_color, text_color):
        self.chat_display.config(state="normal")
        self.chat_display.insert(tk.END, f"{speaker}: ", f"label_{speaker}")
        self.chat_display.insert(tk.END, f"{message}\n", f"text_{speaker}")
        self.chat_display.config(state="disabled")
        self.chat_display.yview(tk.END)

        self.chat_display.tag_config(
            f"label_User", foreground="white", font=("Consolas", 12, "bold")
        )
        self.chat_display.tag_config(
            f"label_TurtleBot Agent",
            foreground=label_color,
            font=("Consolas", 12, "bold"),
        )
        self.chat_display.tag_config(f"text_User", foreground="white")
        self.chat_display.tag_config(f"text_TurtleBot Agent", foreground=text_color)

    def quit_app(self):
        print("Exiting application...")
        self.interface.shutdown()
        self.root.destroy()
        sys.exit(0)
