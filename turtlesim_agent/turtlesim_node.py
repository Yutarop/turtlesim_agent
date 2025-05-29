import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from turtlesim.srv import (Kill, SetPen, Spawn, TeleportAbsolute,
                           TeleportRelative)

from turtlesim_agent.utils import normalize_angle

TWIST_ANGULAR = 0.8
TWIST_VELOCITY = 1.0
ROTATION_ERROR_THRESHOLD = 0.052  # radians
DISTANCE_ERROR_THRESHOLD = 0.1  # centimeters
PUBLISH_RATE = 0.05


class TurtleSimAgent(Node):
    def __init__(self):
        super().__init__("turtlesim_agent")
        self.declare_parameter("interface", "cli")
        self.declare_parameter("agent_model", "gemini-2.0-flash")

        self.interface = (
            self.get_parameter("interface").get_parameter_value().string_value
        )
        self.agent_model = (
            self.get_parameter("agent_model").get_parameter_value().string_value
        )

        # Movement publisher and subscription
        self.pub = self.create_publisher(Twist, f"/turtle1/cmd_vel", 10)
        self.create_subscription(Pose, f"/turtle1/pose", self.pose_callback, 10)
        self.current_pose = None

        # Synchronization
        self.pose_ready = threading.Event()

        # Pen state management
        self._pen_r = 255
        self._pen_g = 255
        self._pen_b = 255
        self._pen_width = 3
        self._pen_off = False

        # Create service clients
        self.reset_client = self.create_client(Empty, "/reset")
        self.clear_client = self.create_client(Empty, "/clear")
        self.kill_client = self.create_client(Kill, "/kill")
        self.spawn_client = self.create_client(Spawn, "/spawn")
        self.set_pen_client = self.create_client(SetPen, "/turtle1/set_pen")
        self.teleport_abs_client = self.create_client(
            TeleportAbsolute, "/turtle1/teleport_absolute"
        )
        self.teleport_rel_client = self.create_client(
            TeleportRelative, "/turtle1/teleport_relative"
        )

    # ===== Pen property accessors =====
    @property
    def pen_r(self):
        """Get the red component of the pen color (0-255)"""
        return self._pen_r

    @property
    def pen_g(self):
        """Get the green component of the pen color (0-255)"""
        return self._pen_g

    @property
    def pen_b(self):
        """Get the blue component of the pen color (0-255)"""
        return self._pen_b

    @property
    def pen_width(self):
        """Get the pen width"""
        return self._pen_width

    @property
    def pen_off(self):
        """Get whether the pen is off (True) or on (False)"""
        return self._pen_off

    @property
    def pen_color(self):
        """Get the pen color as RGB tuple (r, g, b)"""
        return (self._pen_r, self._pen_g, self._pen_b)

    @property
    def pen_info(self):
        """Get complete pen information as dictionary"""
        return {
            "r": self._pen_r,
            "g": self._pen_g,
            "b": self._pen_b,
            "width": self._pen_width,
            "off": self._pen_off,
            "color_rgb": (self._pen_r, self._pen_g, self._pen_b),
            "is_drawing": not self._pen_off,
        }

    def pose_callback(self, msg):
        self.current_pose = msg
        self.pose_ready.set()

    def wait_for_pose(self, timeout=1.0):
        return self.pose_ready.wait(timeout=timeout)

    def _stop_robot(self):
        """Send stop commands to bring the robot to a complete halt."""
        stop_cmd = Twist()
        for _ in range(5):
            self.pub.publish(stop_cmd)
            time.sleep(PUBLISH_RATE)

    # ===== Boundary check method =====
    def is_within_bounds(self, x_min=0, x_max=11, y_min=0, y_max=11):
        """
        Check if the turtle is within the specified bounds.

        Args:
            x_min (float): Minimum x coordinate (default: 0)
            x_max (float): Maximum x coordinate (default: 11)
            y_min (float): Minimum y coordinate (default: 0)
            y_max (float): Maximum y coordinate (default: 11)

        Returns:
            bool: True if turtle is within bounds, False otherwise
        """
        if self.current_pose is None:
            self.get_logger().warning("Current pose is not available")
            return False

        x, y = self.current_pose.x, self.current_pose.y
        within_bounds = (x_min < x < x_max) and (y_min < y < y_max)

        if not within_bounds:
            self.get_logger().info(
                f"Turtle is out of bounds: x={x:.2f}, y={y:.2f} "
                f"(bounds: {x_min}<x<{x_max}, {y_min}<y<{y_max})"
            )

        return within_bounds

    async def check_bounds_and_reset_async(self, x_min=0, x_max=11, y_min=0, y_max=11):
        """
        Check if turtle is within bounds and reset if it's outside.

        Args:
            x_min (float): Minimum x coordinate (default: 0)
            x_max (float): Maximum x coordinate (default: 11)
            y_min (float): Minimum y coordinate (default: 0)
            y_max (float): Maximum y coordinate (default: 11)

        Returns:
            bool: True if turtle was within bounds or reset was successful, False if reset failed
        """
        if not self.wait_for_pose():
            self.get_logger().warning("Failed to get current pose for boundary check")
            return False

        if self.is_within_bounds(x_min, x_max, y_min, y_max):
            return True

        self.get_logger().info("Turtle is out of bounds, calling reset...")
        await self.call_reset_async()
        return True

    # ===== Movement methods =====
    def rotate_angle(self, angle_rad):
        """
        Rotate the robot by a specified angle.

        Args:
            angle_rad: Angle to rotate in radians (positive=counterclockwise)

        Returns:
            bool: True if rotation completed successfully
        """
        if not self.wait_for_pose():
            self.get_logger().warning("Failed to get initial pose for rotation")
            return False

        start_yaw = self.current_pose.theta
        target_yaw = normalize_angle(start_yaw + angle_rad)
        angular_velocity = TWIST_ANGULAR if angle_rad > 0 else -TWIST_ANGULAR

        while rclpy.ok():
            current = self.current_pose.theta
            diff = normalize_angle(target_yaw - current)

            if abs(diff) < ROTATION_ERROR_THRESHOLD:
                break

            twist = Twist()
            twist.angular.z = angular_velocity
            self.pub.publish(twist)
            time.sleep(PUBLISH_RATE)

        self._stop_robot()
        return True

    async def move_linear(self, distance_m):
        """
        Move the robot by a specified distance.

        Args:
            distance_m: Distance to move in centimeters (positive=forward)

        Returns:
            bool: True if movement completed successfully, False if out of bounds
        """
        if not self.wait_for_pose():
            self.get_logger().warning("Failed to get initial pose for movement")
            return False

        start_x, start_y = self.current_pose.x, self.current_pose.y
        self.get_logger().info(
            f"Moving: distance={distance_m}cm from ({start_x}, {start_y})"
        )

        linear_velocity = TWIST_VELOCITY if distance_m > 0 else -TWIST_VELOCITY

        while rclpy.ok():
            # Check bounds and reset if out of bounds
            if not self.is_within_bounds():
                self.get_logger().warning(
                    "Turtle went out of bounds during linear movement"
                )
                await self.call_reset_async()
                self._stop_robot()
                return False

            dx = self.current_pose.x - start_x
            dy = self.current_pose.y - start_y
            moved_distance = math.sqrt(dx**2 + dy**2)
            remaining = abs(distance_m) - moved_distance

            if remaining < DISTANCE_ERROR_THRESHOLD:
                break

            twist = Twist()
            twist.linear.x = linear_velocity
            self.pub.publish(twist)
            time.sleep(PUBLISH_RATE)

        self._stop_robot()
        return True

    async def move_non_linear(
        self,
        duration_sec,
        linear_velocity=TWIST_VELOCITY,
        angular_velocity=TWIST_ANGULAR,
    ):
        """
        Move the robot with specified linear and angular velocities for a certain duration.

        Args:
            linear_velocity (float): Linear velocity in cm/s. Positive for forward.
            angular_velocity (float): Angular velocity in rad/s. Positive for left turn.
            duration_sec (float): Time in seconds to apply the twist.

        Returns:
            bool: True if movement executed successfully, False if out of bounds
        """
        if not self.wait_for_pose():
            self.get_logger().warning("Failed to get initial pose for curve movement")
            return False

        self.get_logger().info(
            f"Moving in a curve: linear_velocity={linear_velocity} cm/s, "
            f"angular_velocity={angular_velocity} rad/s, duration={duration_sec} sec"
        )

        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time < duration_sec):
            # Check bounds and reset if out of bounds
            if not self.is_within_bounds():
                self.get_logger().warning(
                    "Turtle went out of bounds during non-linear movement"
                )
                await self.call_reset_async()
                self._stop_robot()
                return False

            twist = Twist()
            twist.linear.x = linear_velocity
            twist.angular.z = angular_velocity
            self.pub.publish(twist)
            time.sleep(PUBLISH_RATE)

        self._stop_robot()
        return True

    # ===== Service client methods =====
    def wait_for_service(self, client):
        """Wait for a service to become available."""
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

    async def call_reset_async(self):
        """Reset the turtle simulation."""
        self.wait_for_service(self.reset_client)
        request = Empty.Request()
        future = self.reset_client.call_async(request)
        await future
        if future.result() is not None:
            self.get_logger().info("Reset the turtle")
            # Reset pen to default values after reset
            self._pen_r = 0
            self._pen_g = 0
            self._pen_b = 255
            self._pen_width = 3
            self._pen_off = False
        else:
            self.get_logger().error("Failed to reset turtle")

    async def call_clear_async(self):
        """Clear the turtlesim screen."""
        self.wait_for_service(self.clear_client)
        request = Empty.Request()
        future = self.clear_client.call_async(request)
        await future
        if future.result() is not None:
            self.get_logger().info("Cleared the screen")
        else:
            self.get_logger().error("Failed to clear screen")

    async def call_kill_async(self, name):
        """Kill a turtle by name."""
        self.wait_for_service(self.kill_client)
        request = Kill.Request()
        request.name = name
        future = self.kill_client.call_async(request)
        await future
        if future.result() is not None:
            self.get_logger().info(f"Killed turtle: {name}")
        else:
            self.get_logger().error(f"Failed to kill turtle: {name}")

    async def call_spawn_async(self, x, y, theta, name=""):
        """Spawn a new turtle at specified position and orientation."""
        self.wait_for_service(self.spawn_client)
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        future = self.spawn_client.call_async(request)
        await future
        if future.result() is not None:
            self.get_logger().info(f"Spawned turtle: {future.result().name}")
            return future.result().name
        else:
            self.get_logger().error("Failed to spawn turtle")
            return None

    async def call_set_pen_async(self, r, g, b, width, off):
        """Set the pen properties for drawing."""
        self.wait_for_service(self.set_pen_client)
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        future = self.set_pen_client.call_async(request)
        await future
        if future.result() is not None:
            # Update internal pen state after successful service call
            self._pen_r = r
            self._pen_g = g
            self._pen_b = b
            self._pen_width = width
            self._pen_off = off
            self.get_logger().info(
                f"Set pen: R={r}, G={g}, B={b}, width={width}, off={off}"
            )
        else:
            self.get_logger().error("Failed to set pen")

    async def call_teleport_absolute_async(self, x, y, theta):
        """Teleport turtle to absolute position and orientation."""
        self.wait_for_service(self.teleport_abs_client)
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        future = self.teleport_abs_client.call_async(request)
        await future
        if future.result() is not None:
            self.get_logger().info(f"Teleported to: x={x}, y={y}, theta={theta}")
        else:
            self.get_logger().error("Failed to teleport")

    async def call_teleport_relative_async(self, linear, angular):
        """Teleport turtle relative to current position."""
        self.wait_for_service(self.teleport_rel_client)
        request = TeleportRelative.Request()
        request.linear = linear
        request.angular = angular
        future = self.teleport_rel_client.call_async(request)
        await future
        if future.result() is not None:
            self.get_logger().info(
                f"Relative teleport: linear={linear}, angular={angular}"
            )
        else:
            self.get_logger().error("Failed to teleport")
