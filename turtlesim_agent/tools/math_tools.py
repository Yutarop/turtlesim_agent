#!/usr/bin/env python3

import math

from langchain.tools import tool

TWIST_ANGULAR = 0.8
TWIST_VELOCITY = 1.0


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


@tool
def compute_linear_and_angular_velocity_from_radius(
    radius: float,
) -> list[float]:
    """
    Compute linear velocity (v) and angular velocity (ω) for a circular trajectory based on the given radius (r).

    Args:
        radius (float): Radius of the circular path (in centimeters)

    Returns:
        list[float]: [linear_velocity (cm/s), angular_velocity (rad/s)]
    """
    if radius > 0.7:
        linear_velocity = TWIST_VELOCITY
        angular_velocity = linear_velocity / radius
    else:
        angular_velocity = TWIST_ANGULAR
        linear_velocity = radius * angular_velocity

    return [linear_velocity, angular_velocity]


@tool
def compute_duration_from_circular_angle_and_angular_velocity(
    circular_angle: float, angular_velocity: float
) -> float:
    """
    Compute the time required to rotate a given angle at a specified angular velocity.

    Formula: time = angle / angular_velocity

    Args:
        circular_angle (float): The angle to rotate along the circular path (in radians)
        angular_velocity (float): Angular velocity for rotation (in rad/s)

    Returns:
        float: Duration in seconds needed to rotate the specified angle
    """
    duration = abs(circular_angle / angular_velocity)
    return round(duration, 3)
