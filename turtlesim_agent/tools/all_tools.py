from turtlesim_agent.tools.math_tools import (
    calculate_euclidean_distance,
    degrees_to_radians,
)
from turtlesim_agent.tools.tools import (
    make_check_bounds_tool,
    make_clear_tool,
    make_get_pen_status_tool,
    make_get_turtle_pose_tool,
    make_kill_tool,
    make_move_linear_tool,
    make_move_non_linear_tool,
    make_reset_tool,
    make_rotate_tool,
    make_set_pen_color_width_tool,
    make_set_pen_up_down_tool,
    make_spawn_tool,
    make_teleport_absolute_tool,
    make_teleport_relative_tool,
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
        degrees_to_radians,
        calculate_euclidean_distance,
        make_check_bounds_tool(node),
        make_get_turtle_pose_tool(node),
        make_get_pen_status_tool(node),
        make_move_linear_tool(node),
        make_move_non_linear_tool(node),
        make_rotate_tool(node),
        make_reset_tool(node),
        make_clear_tool(node),
        make_kill_tool(node),
        make_spawn_tool(node),
        make_set_pen_color_width_tool(node),
        make_set_pen_up_down_tool(node),
        make_teleport_absolute_tool(node),
        make_teleport_relative_tool(node),
    ]
