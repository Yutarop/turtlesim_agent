#!/usr/bin/env python3

import math

from langchain.tools import tool


@tool
def degrees_to_radians(degrees: float) -> float:
    """
    Converts an angle in degrees to radians.

    Args:
        degrees (float): The angle in degrees.

    Returns:
        float: The angle in radians.
    """
    radians = round(math.radians(degrees), 3)
    return radians


@tool
def calculate_euclidean_distance(
    current_x: float,
    current_y: float,
    target_x: float,
    target_y: float,
) -> float:
    """
    Calculates the Euclidean distance between the current position and the target position.

    Args:
        current_x (float): X coordinate of the current position.
        current_y (float): Y coordinate of the current position.
        target_x (float): X coordinate of the target position.
        target_y (float): Y coordinate of the target position.

    Returns:
        float: Distance in meters.
    """
    dx = target_x - current_x
    dy = target_y - current_y
    return round(math.sqrt(dx**2 + dy**2), 3)
