#!/usr/bin/env python3

"""
TurtleSim Tools Package

This package provides LangChain-compatible tools for controlling TurtleSim.
Tools are organized by functionality:
- status: Information retrieval tools (pose, pen status, bounds checking)
- motion: Movement and rotation tools
- pen: Drawing tools (pen control, color, width)
- simulation: Simulation control tools (reset, clear, spawn, kill, teleport)
"""

from turtlesim_agent.tools.math_tools import (
    calculate_euclidean_distance,
    compute_duration_from_circular_angle_and_angular_velocity,
    compute_linear_and_angular_velocity_from_radius,
    degrees_to_radians,
)
from turtlesim_agent.tools.motion_tools import (
    make_move_linear_tool,
    make_move_non_linear_tool,
    make_move_on_arc_tool,
    make_rotate_tool,
    make_teleport_absolute_tool,
    make_teleport_relative_tool,
)
from turtlesim_agent.tools.pen_tools import (
    make_set_pen_color_width_tool,
    make_set_pen_up_down_tool,
)
from turtlesim_agent.tools.sim_tools import (
    make_clear_tool,
    make_kill_tool,
    make_reset_tool,
    make_spawn_tool,
)
from turtlesim_agent.tools.status_tools import (
    make_check_bounds_tool,
    make_get_pen_status_tool,
    make_get_turtle_pose_tool,
)


def make_all_tools(node) -> list:
    """
    Creates all the tools needed for the LangChain agent.

    Args:
        node: The TurtleSimAgent node instance

    Returns:
        list: List of tools available to the agent
    """
    return [
        # Math tools
        degrees_to_radians,
        calculate_euclidean_distance,
        compute_duration_from_circular_angle_and_angular_velocity,
        compute_linear_and_angular_velocity_from_radius,
        make_move_on_arc_tool(node),
        # Status tools
        make_check_bounds_tool(node),
        make_get_turtle_pose_tool(node),
        make_get_pen_status_tool(node),
        # Motion tools
        make_move_linear_tool(node),
        make_move_non_linear_tool(node),
        make_rotate_tool(node),
        make_teleport_absolute_tool(node),
        make_teleport_relative_tool(node),
        # Pen tools
        make_set_pen_color_width_tool(node),
        make_set_pen_up_down_tool(node),
        # Simulation tools
        make_reset_tool(node),
        make_clear_tool(node),
        make_kill_tool(node),
        make_spawn_tool(node),
    ]
