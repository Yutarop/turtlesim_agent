#!/usr/bin/env python3

"""
Pen Tools for TurtleSim

This module contains tools for controlling the turtle's drawing pen,
including color, width, and up/down state management.
"""

from langchain.tools import StructuredTool
from pydantic import BaseModel, Field
from pydantic.v1 import BaseModel, Field


def make_set_pen_color_width_tool(node):
    """
    Create a tool that sets the pen color and width while maintaining current pen state (up/down).

    Args:
        node: The TurtleSimAgent node instance

    Returns:
        StructuredTool: A LangChain tool for setting pen color and width
    """

    class SetPenColorWidthInput(BaseModel):
        r: int = Field(description="Red color component (0-255)")
        g: int = Field(description="Green color component (0-255)")
        b: int = Field(description="Blue color component (0-255)")
        width: int = Field(description="Pen width (1-10)")

    async def inner(r: int, g: int, b: int, width: int) -> str:
        # Use current pen up/down state
        current_pen_off = node.pen_off
        await node.call_set_pen_async(r, g, b, width, current_pen_off)

        pen_state = "up (not drawing)" if current_pen_off else "down (drawing)"
        return f"Pen color set to RGB({r}, {g}, {b}) with width {width}. Pen is {pen_state}."

    return StructuredTool.from_function(
        func=inner,
        coroutine=inner,
        name="set_pen_color_width",
        description="""
        Sets the pen color and width while maintaining the current pen state (up/down).

        Args:
        - r (int): Red color component (0-255).
        - g (int): Green color component (0-255).
        - b (int): Blue color component (0-255).
        - width (int): Pen width (1-10).

        The pen's current up/down state will be preserved.
        """,
        args_schema=SetPenColorWidthInput,
    )


def make_set_pen_up_down_tool(node):
    """
    Create a tool that sets the pen up or down while maintaining current color and width.

    Args:
        node: The TurtleSimAgent node instance

    Returns:
        StructuredTool: A LangChain tool for setting pen up/down
    """

    class SetPenUpDownInput(BaseModel):
        pen_up: bool = Field(
            description="Set pen up (True) to stop drawing, or down (False) to start drawing"
        )

    async def inner(pen_up: bool) -> str:
        # Use current pen color and width
        current_r = node.pen_r
        current_g = node.pen_g
        current_b = node.pen_b
        current_width = node.pen_width

        await node.call_set_pen_async(
            current_r, current_g, current_b, current_width, pen_up
        )

        if pen_up:
            return f"Pen lifted up - turtle will not draw trails when moving. Color: RGB({current_r}, {current_g}, {current_b}), Width: {current_width}"
        else:
            return f"Pen put down - turtle will draw trails when moving. Color: RGB({current_r}, {current_g}, {current_b}), Width: {current_width}"

    return StructuredTool.from_function(
        func=inner,
        coroutine=inner,
        name="set_pen_up_down",
        description="""
        Sets the pen up (stop drawing) or down (start drawing) while maintaining current color and width.

        Args:
        - pen_up (bool): Set to True to lift pen up (stop drawing), False to put pen down (start drawing).

        The pen's current color and width settings will be preserved.
        """,
        args_schema=SetPenUpDownInput,
    )
