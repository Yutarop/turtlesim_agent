#!/usr/bin/env python3

import asyncio
import math

from langchain.tools import StructuredTool, Tool, tool
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


def make_move_linear_tool(node):
    """
    Create a tool that publishes Twist commands to move the TurtleSim.

    Args:
        node: The TurtleSim node instance

    Returns:
        StructuredTool: A LangChain tool for sending movement commands
    """

    class MoveInput(BaseModel):
        distance: float = Field(
            description="Linear distance in centimeters. Positive for forward, negative for backward."
        )

    async def inner(distance: float) -> str:
        if distance != 0.0:
            result = await node.move_linear(distance_m=distance)
            # Check if movement was interrupted due to out of bounds
            if not result:
                return "Turtle went out of bounds during movement and has been reset to the starting position."

        if not node.wait_for_pose(timeout=0.5):
            return "Pose data not received yet."
        p = node.current_pose
        return (
            f"Current Pose of the turtlebot: x={p.x:.2f}, y={p.y:.2f}, "
            f"yaw={p.theta:.2f} rad, "
        )

    return StructuredTool.from_function(
        func=inner,
        coroutine=inner,
        name="move_linear",
        description="""
        Moves the TurtleSim forward or backward by the specified distance.

        Args:
        - distance (float): Linear distance in centimeters. Positive for forward, negative for backward. Default value is 2.0.
        """,
        args_schema=MoveInput,
    )


def make_rotate_tool(node):
    """
    Create a tool that publishes Twist commands to rotate the TurtleSim.

    Args:
        node: The TurtleSim node instance

    Returns:
        StructuredTool: A LangChain tool for sending movement commands
    """

    class RotateInput(BaseModel):
        angle: float = Field(
            description="Relative rotation in radians. Positive to turn left (counterclockwise), negative to turn right (clockwise)."
        )

    def inner(angle: float) -> str:
        if angle != 0.0:
            node.rotate_angle(angle_rad=angle)

        if not node.wait_for_pose(timeout=0.5):
            return "Pose data not received yet."
        p = node.current_pose
        return (
            f"Current Pose of the turtlebot: x={p.x:.2f}, y={p.y:.2f}, "
            f"yaw={p.theta:.2f} rad, "
        )

    return StructuredTool.from_function(
        func=inner,
        name="rotate_robot",
        description="""
        Rotates the TurtleSim by the specified angle (in radians).

        Args:
        - angle (float): Relative rotation in radians. Positive for left (counterclockwise), negative for right (clockwise).
        """,
        args_schema=RotateInput,
    )


def make_move_non_linear_tool(node):
    """
    Create a LangChain tool for executing curved movement.

    Args:
        node: The TurtleSim node instance

    Returns:
        StructuredTool: LangChain-compatible tool
    """

    class MoveCurveInput(BaseModel):
        linear_velocity: float = Field(
            description="Forward/backward speed in cm/s. Positive to move forward, negative to move backward."
        )
        angular_velocity: float = Field(
            description="Rotational speed in rad/s. Positive to turn left (counterclockwise), negative to turn right (clockwise)."
        )
        duration_sec: float = Field(description="Duration of movement in seconds.")

    async def inner(
        linear_velocity: float, angular_velocity: float, duration_sec: float
    ) -> str:
        result = await node.move_non_linear(
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
            duration_sec=duration_sec,
        )
        # Check if movement was interrupted due to out of bounds
        if not result:
            return "Turtle went out of bounds during curved movement and has been reset to the starting position."

        if not node.wait_for_pose(timeout=0.5):
            return "Pose data not received yet."
        p = node.current_pose
        return (
            f"Current Pose of the turtlebot: x={p.x:.2f}, y={p.y:.2f}, "
            f"yaw={p.theta:.2f} rad, "
        )

    return StructuredTool.from_function(
        func=inner,
        coroutine=inner,
        name="move_non_linear",
        description="""
        Moves the TurtleSim in a curved trajectory.

        Args:
        - linear_velocity (cm/s): Forward/backward speed. Positive for forward motion. Should be less than 1.0cm/s.
        - angular_velocity (rad/s): Rotational speed. Positive for left (CCW) turns. Should be less than 0.5rad/s.
        - duration_sec (s): How long the motion lasts.

        Use this to make the robot move in arcs or spirals.
        """,
        args_schema=MoveCurveInput,
    )


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


def make_teleport_absolute_tool(node):
    """
    Create a tool that teleports the turtle to an absolute position.

    Args:
        node: The TurtleSimAgent node instance

    Returns:
        StructuredTool: A LangChain tool for absolute teleportation
    """
    set_pen_tool = make_set_pen_up_down_tool(node)

    class TeleportAbsInput(BaseModel):
        x: float = Field(description="Target X position (0.0-11.0)")
        y: float = Field(description="Target Y position (0.0-11.0)")
        theta: float = Field(description="Target orientation in radians")

    async def inner(x: float, y: float, theta: float) -> str:
        current_r = node.pen_r
        current_g = node.pen_g
        current_b = node.pen_b
        current_width = node.pen_width

        await node.call_set_pen_async(
            current_r, current_g, current_b, current_width, True
        )
        await node.call_teleport_absolute_async(x, y, theta)
        await node.call_set_pen_async(
            current_r, current_g, current_b, current_width, False
        )
        return f"Turtle teleported to position ({x}, {y}) with orientation {theta} rad."

    return StructuredTool.from_function(
        func=inner,
        coroutine=inner,
        name="teleport_absolute",
        description="""
        Teleports the turtle to an absolute position and orientation instantly without drawing a trail. 
        After teleporting, the pen is down.

        Args:
        - x (float): Target X position (0.0-11.0).
        - y (float): Target Y position (0.0-11.0).
        - theta (float): Target orientation in radians.
        """,
        args_schema=TeleportAbsInput,
    )


def make_teleport_relative_tool(node):
    """
    Create a tool that teleports the turtle relative to its current position.

    Args:
        node: The TurtleSimAgent node instance

    Returns:
        StructuredTool: A LangChain tool for relative teleportation
    """

    class TeleportRelInput(BaseModel):
        linear: float = Field(description="Linear distance to teleport forward")
        angular: float = Field(description="Angular rotation in radians")

    async def inner(linear: float, angular: float) -> str:
        current_r = node.pen_r
        current_g = node.pen_g
        current_b = node.pen_b
        current_width = node.pen_width

        await node.call_set_pen_async(
            current_r, current_g, current_b, current_width, True
        )
        await node.call_teleport_relative_async(linear, angular)
        await node.call_set_pen_async(
            current_r, current_g, current_b, current_width, False
        )
        return (
            f"Turtle teleported {linear} units forward and rotated {angular} radians."
        )

    return StructuredTool.from_function(
        func=inner,
        coroutine=inner,
        name="teleport_relative",
        description="""
        Teleports the turtle relative to its current position and orientation instantly without drawing a trail.

        Args:
        - linear (float): Linear distance to teleport forward from current position.
        - angular (float): Angular rotation in radians from current orientation.
        """,
        args_schema=TeleportRelInput,
    )
