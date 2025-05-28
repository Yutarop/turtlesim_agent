import asyncio
import math
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from turtlesim_agent.turtlesim_node import TurtleSimAgent


def spin_node_thread(node):
    """
    Run the ROS2 node in a separate thread to allow concurrent execution.

    Args:
        node: The ROS2 node to spin
    """
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()


async def main():
    rclpy.init()
    node = TurtleSimAgent()

    spin_thread = threading.Thread(target=spin_node_thread, args=(node,), daemon=True)
    spin_thread.start()

    if not node.wait_for_pose(timeout=3.0):
        node.get_logger().error("Initial pose not received")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.move_linear(2.0)
    await node.call_set_pen_async(0, 0, 255, 3, False)
    node.move_linear(2.0)
    await node.call_reset_async()
    print("finish")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
