#!/usr/bin/env python3

"""
Status Tools for TurtleSim

This module contains tools for retrieving status information about the turtle,
including position, pen status, and boundary checking.
"""

from langchain.tools import StructuredTool, Tool
from pydantic import BaseModel, Field
from pydantic.v1 import BaseModel, Field


def make_get_turtle_pose_tool(node):
    """
    Create a tool that returns the current pose of the TurtleSim.

    Args:
        node: The TurtleSim node instance

    Returns:
        Tool: A LangChain tool for getting the robot's pose
    """

    def inner(_input: str = "") -> str:
        """
        Returns the current pose of the TurtleSim by subscribing to the /odom topic.
        The input is unused but required by LangChain.
        """
        if not node.wait_for_pose(timeout=0.5):
            result = "Could not retrieve pose for the turtle bot."
        else:
            p = node.current_pose
        result = (
            f"Current Pose of the turtlebot: x={p.x:.2f}, y={p.y:.2f}, "
            f"yaw={p.theta:.2f} rad, "
        )
        return result

    return Tool.from_function(
        func=inner,
        name="get_turtle_pose",
        description="""
        Returns the current pose of the TurtleSim by subscribing to the /odom topic.

        Args:
        - _input: the input is unused but required by LangChain. Set an empty string "".

        Returns:
        - Position: x (centimeters), y (centimeters)
        - Orientation: yaw (in radians)
        """,
    )


def make_get_pen_status_tool(node):
    """
    Create a tool that returns the current pen status and properties.

    Args:
        node: The TurtleSimAgent node instance

    Returns:
        Tool: A LangChain tool for getting the pen status
    """

    def inner(_input: str = "") -> str:
        """
        Returns the current pen status and properties.
        The input is unused but required by LangChain.
        """
        pen_info = node.pen_info
        status = "up (not drawing)" if pen_info["off"] else "down (drawing)"

        result = (
            f"Current Pen Status:\n"
            f"- State: {status}\n"
            f"- Color: RGB({pen_info['r']}, {pen_info['g']}, {pen_info['b']})\n"
            f"- Width: {pen_info['width']}\n"
            f"- Drawing: {'No' if pen_info['off'] else 'Yes'}"
        )
        return result

    return Tool.from_function(
        func=inner,
        name="get_pen_status",
        description="""
        Returns the current pen status and properties including color, width, and drawing state.

        Args:
        - _input: the input is unused but required by LangChain. Set an empty string "".

        Returns:
        - Current pen state (up/down)
        - Color values (RGB)
        - Pen width
        - Whether the pen is currently drawing
        """,
    )


def make_check_bounds_tool(node):
    """
    Create a tool that checks if the turtle is within specified boundaries.

    Args:
        node: The TurtleSimAgent node instance

    Returns:
        StructuredTool: A LangChain tool for checking turtle boundaries
    """

    class CheckBoundsInput(BaseModel):
        x_min: float = Field(
            default=0.0, description="Minimum x coordinate (default: 0.0)"
        )
        x_max: float = Field(
            default=11.0, description="Maximum x coordinate (default: 11.0)"
        )
        y_min: float = Field(
            default=0.0, description="Minimum y coordinate (default: 0.0)"
        )
        y_max: float = Field(
            default=11.0, description="Maximum y coordinate (default: 11.0)"
        )

    def inner(
        x_min: float = 0.0, x_max: float = 11.0, y_min: float = 0.0, y_max: float = 11.0
    ) -> str:
        """
        Checks if the turtle is within the specified boundaries.
        """
        if not node.wait_for_pose(timeout=0.5):
            return "Could not retrieve pose to check boundaries."

        is_within = node.is_within_bounds(
            x_min=x_min,
            x_max=x_max,
            y_min=y_min,
            y_max=y_max,
        )

        current_pos = node.current_pose
        if is_within:
            result = (
                f"Turtle is WITHIN bounds.\n"
                f"Current position: x={current_pos.x:.2f}, y={current_pos.y:.2f}\n"
                f"Allowed bounds: {x_min} < x < {x_max}, {y_min} < y < {y_max}"
            )
        else:
            result = (
                f"Turtle is OUT of bounds!\n"
                f"Current position: x={current_pos.x:.2f}, y={current_pos.y:.2f}\n"
                f"Allowed bounds: {x_min} < x < {x_max}, {y_min} < y < {y_max}"
            )

        return result

    return StructuredTool.from_function(
        func=inner,
        name="check_turtle_bounds",
        description="""
        Checks if the turtle is within the specified boundaries.

        Args:
        - x_min (float): Minimum x coordinate (default: 0.0)
        - x_max (float): Maximum x coordinate (default: 11.0)
        - y_min (float): Minimum y coordinate (default: 0.0)
        - y_max (float): Maximum y coordinate (default: 11.0)

        Returns:
        - Whether the turtle is within or outside the specified bounds
        - Current turtle position
        - The boundary limits being checked
        """,
        args_schema=CheckBoundsInput,
    )
