#!/usr/bin/env python3

"""
Simulation Tools for TurtleSim

This module contains tools for controlling the TurtleSim simulation,
including reset, clear, spawn, kill, and teleportation functions.
"""

from langchain.tools import StructuredTool, Tool
from pydantic import BaseModel, Field
from pydantic.v1 import BaseModel, Field


def make_reset_tool(node):
    """
    Create a tool that resets the turtle simulation.

    Args:
        node: The TurtleSimAgent node instance

    Returns:
        Tool: A LangChain tool for resetting the simulation
    """

    async def inner(_input: str = "") -> str:
        """
        Resets the turtle simulation to initial state.
        The input is unused but required by LangChain.
        """
        await node.call_reset_async()
        return "Turtle simulation has been reset to initial state."

    return Tool.from_function(
        func=inner,
        coroutine=inner,
        name="reset_simulation",
        description="""
        Resets the turtle simulation to its initial state.

        Args:
        - _input: the input is unused but required by LangChain. Set an empty string "".

        Returns:
        - Confirmation message that the simulation has been reset.
        """,
    )


def make_clear_tool(node):
    """
    Create a tool that clears the turtlesim screen.

    Args:
        node: The TurtleSimAgent node instance

    Returns:
        Tool: A LangChain tool for clearing the screen
    """

    async def inner(_input: str = "") -> str:
        """
        Clears the turtlesim screen of all drawing traces.
        The input is unused but required by LangChain.
        """
        await node.call_clear_async()
        return "Screen has been cleared of all drawing traces."

    return Tool.from_function(
        func=inner,
        coroutine=inner,
        name="clear_screen",
        description="""
        Clears the turtlesim screen of all drawing traces.

        Args:
        - _input: the input is unused but required by LangChain. Set an empty string "".

        Returns:
        - Confirmation message that the screen has been cleared.
        """,
    )


def make_kill_tool(node):
    """
    Create a tool that kills a turtle by name.

    Args:
        node: The TurtleSimAgent node instance

    Returns:
        StructuredTool: A LangChain tool for killing turtles
    """

    class KillInput(BaseModel):
        name: str = Field(
            description="Name of the turtle to kill (e.g., 'turtle1', 'turtle2')"
        )

    async def inner(name: str) -> str:
        await node.call_kill_async(name)
        return f"Turtle '{name}' has been killed."

    return StructuredTool.from_function(
        func=inner,
        coroutine=inner,
        name="kill_turtle",
        description="""
        Kills a turtle by its name, removing it from the simulation.

        Args:
        - name (str): Name of the turtle to kill (e.g., 'turtle1', 'turtle2').
        """,
        args_schema=KillInput,
    )


def make_spawn_tool(node):
    """
    Create a tool that spawns a new turtle.

    Args:
        node: The TurtleSimAgent node instance

    Returns:
        StructuredTool: A LangChain tool for spawning turtles
    """

    class SpawnInput(BaseModel):
        x: float = Field(description="X position to spawn the turtle (0.0-11.0)")
        y: float = Field(description="Y position to spawn the turtle (0.0-11.0)")
        theta: float = Field(description="Initial orientation in radians")
        name: str = Field(default="", description="Name for the new turtle (optional)")

    async def inner(x: float, y: float, theta: float, name: str = "") -> str:
        spawned_name = await node.call_spawn_async(x, y, theta, name)
        if spawned_name:
            return f"New turtle '{spawned_name}' spawned at position ({x}, {y}) with orientation {theta} rad."
        else:
            return "Failed to spawn new turtle."

    return StructuredTool.from_function(
        func=inner,
        coroutine=inner,
        name="spawn_turtle",
        description="""
        Spawns a new turtle at the specified position and orientation.

        Args:
        - x (float): X position to spawn the turtle (0.0-11.0).
        - y (float): Y position to spawn the turtle (0.0-11.0).
        - theta (float): Initial orientation in radians.
        - name (str): Optional name for the new turtle. If empty, a name will be auto-generated.
        """,
        args_schema=SpawnInput,
    )
