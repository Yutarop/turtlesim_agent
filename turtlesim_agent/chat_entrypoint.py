import asyncio
import sys
import threading
import tkinter as tk

import rclpy
from langchain.agents import AgentExecutor
from rclpy.node import Node

from turtlesim_agent.interface.chat_gui import ChatUI
from turtlesim_agent.interface.gui_interface import GUIAgentInterface


def chat_invoke(
    interface: str,
    agent_executor: AgentExecutor,
    node: Node,
    spin_thread: threading.Thread,
):
    if interface == "cli":
        asyncio.run(_run_cli(agent_executor, node, spin_thread))
    elif interface == "gui":
        asyncio.run(_run_gui(agent_executor, node))
    else:
        raise ValueError(f"Unsupported interface: {interface}")


async def _run_cli(
    agent_executor: AgentExecutor, node: Node, spin_thread: threading.Thread
):
    try:
        while True:
            user_input = input("user: ")
            if user_input.lower() in {"quit", "exit"}:
                break
            result = await agent_executor.ainvoke(input={"input": user_input})
            print("turtlebot agent:", result["output"])
    except KeyboardInterrupt:
        node.get_logger().info("User interrupted. Shutting down...")
    finally:
        _shutdown(node, spin_thread)


def _run_gui(agent_executor: AgentExecutor, node: Node):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    interface = GUIAgentInterface(agent_executor)

    def start_loop():
        asyncio.set_event_loop(loop)
        loop.run_forever()

    loop_thread = threading.Thread(target=start_loop, daemon=True)
    loop_thread.start()

    def on_close():
        interface.shutdown()
        _shutdown(node)
        root.destroy()
        loop.call_soon_threadsafe(loop.stop)
        sys.exit(0)

    try:
        root = tk.Tk()
        ChatUI(root, interface, loop)
        root.protocol("WM_DELETE_WINDOW", on_close)
        root.mainloop()
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Exiting.")
        on_close()


def _shutdown(node: Node, spin_thread: threading.Thread = None):
    node.destroy_node()
    rclpy.shutdown()
    if spin_thread and spin_thread.is_alive():
        spin_thread.join(timeout=1.0)
