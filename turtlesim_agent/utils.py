#!/usr/bin/env python3
"""
Utility functions for the Turtlesim agent.
Contains helper functions used across the project.
"""

import math


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to the range [-π, π].

    Args:
        angle (float): The input angle in radians

    Returns:
        float: The normalized angle in radians
    """
    return math.atan2(math.sin(angle), math.cos(angle))
