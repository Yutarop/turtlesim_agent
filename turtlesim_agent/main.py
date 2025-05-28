#!/usr/bin/env python3
"""
Main entry point for the TurtleSim agent with LangChain integration.
This module initializes the ROS2 node and connects it with a language-based agent
that can interpret natural language commands to control the robot.
"""

import threading

import rclpy
from langchain.agents import AgentExecutor
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from turtlesim_agent.chat_entrypoint import chat_invoke
from turtlesim_agent.llms import create_agent
from turtlesim_agent.tools.all_tools import make_all_tools
from turtlesim_agent.turtlesim_node import TurtleSimAgent


def spin_node_thread(node):
    """
    Run the ROS2 node in a separate thread to allow concurrent execution.

    Args:
        node: The ROS2 node to spin
    """
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()


def main():
    """
    Main function that initializes and runs the TurtleSim agent with LangChain integration.
    """

    rclpy.init()
    node = TurtleSimAgent()

    spin_thread = threading.Thread(target=spin_node_thread, args=(node,), daemon=True)
    spin_thread.start()

    if not node.wait_for_pose(timeout=3.0):
        node.get_logger().error("Initial pose not received")
        node.destroy_node()
        rclpy.shutdown()
        return

    tools = make_all_tools(node)
    agent = create_agent(model_name="gemini-2.0-flash", tools=tools, temperature=0.0)
    agent_executor = AgentExecutor(agent=agent, tools=tools, verbose=True)
    chat_invoke(
        interface="cli",
        agent_executor=agent_executor,
        node=node,
        spin_thread=spin_thread,
    )


if __name__ == "__main__":
    main()
