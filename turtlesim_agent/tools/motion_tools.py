#!/usr/bin/env python3

"""
Motion Tools for TurtleSim

This module contains tools for controlling turtle movement including
linear movement, rotation, and curved trajectories.
"""


from langchain.tools import StructuredTool
from pydantic import BaseModel, Field
from pydantic.v1 import BaseModel, Field

from turtlesim_agent.tools.math_tools import (
    compute_duration_from_circular_angle_and_angular_velocity,
    compute_linear_and_angular_velocity_from_radius,
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
        - linear_velocity (cm/s): Forward/backward speed. Positive for forward motion.
        - angular_velocity (rad/s): Rotational speed. Positive for left (CCW) turns.
        - duration_sec (s): How long the motion lasts.

        Use this to make the robot move in arcs or spirals.
        """,
        args_schema=MoveCurveInput,
    )


def make_teleport_absolute_tool(node):
    """
    Create a tool that teleports the turtle to an absolute position.

    Args:
        node: The TurtleSimAgent node instance

    Returns:
        StructuredTool: A LangChain tool for absolute teleportation
    """

    class TeleportAbsInput(BaseModel):
        x: float = Field(description="Target X position (0.0-11.0)")
        y: float = Field(description="Target Y position (0.0-11.0)")
        yaw: float = Field(description="Target orientation in radians")

    async def inner(x: float, y: float, yaw: float) -> str:
        current_r = node.pen_r
        current_g = node.pen_g
        current_b = node.pen_b
        current_width = node.pen_width

        await node.call_set_pen_async(
            current_r, current_g, current_b, current_width, True
        )
        await node.call_teleport_absolute_async(x, y, yaw)
        await node.call_set_pen_async(
            current_r, current_g, current_b, current_width, False
        )
        return f"Turtle teleported to position ({x}, {y}) with orientation {yaw} rad."

    return StructuredTool.from_function(
        func=inner,
        coroutine=inner,
        name="teleport_absolute",
        description="""
        Teleports the turtle to an absolute position and absolute orientation instantly without drawing a trail.
        The pen is automatically lifted up before teleporting and lowered afterward so don't have to use set_pen_up_down tool at the same time.
        Example: If you put {'x': 5.0, 'y': 5.0, 'yaw': 1.57}, you will be at the position of (5.0, 5.0) facing north.

        Args:
        - x (float): Target X position (0.0-11.0).
        - y (float): Target Y position (0.0-11.0).
        - yaw (float): Target yaw in radians.
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


def make_move_on_arc_tool(node):
    """
    Create a tool that moves turtle along a circular arc using radius and angle.

    Args:
        node: The TB3Agent node instance

    Returns:
        StructuredTool: LangChain tool to move turtle along an arc.
    """

    class ArcMoveInput(BaseModel):
        radius: float = Field(description="Radius of the circular path in centimeters")
        circular_angle: float = Field(
            description="Angle to rotate along the arc in radians"
        )

    async def inner(radius: float, circular_angle: float) -> str:
        linear_velocity, angular_velocity = (
            compute_linear_and_angular_velocity_from_radius.invoke({"radius": radius})
        )

        duration = compute_duration_from_circular_angle_and_angular_velocity.invoke(
            {"circular_angle": circular_angle, "angular_velocity": angular_velocity}
        )

        if circular_angle < 0:
            angular_velocity = -angular_velocity

        result = await node.move_non_linear(
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
            duration_sec=duration,
        )
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
        name="move_on_arc",
        description="""
        Moves the turtle along a circular arc using the specified radius and angle.
        Example: If you want to draw a semi-circular arc from left to right, you need to face north and set the circular_angle parameter to -3.14.
        Conversely, if you want to draw a semi-circular arc from right to left, you need to face north and set circular_angle to +3.14.

        Args:
            radius (float): radius of the circular path in centimeters
            circular_angle (float) : angle to rotate along the arc in radians
        """,
        args_schema=ArcMoveInput,
    )
