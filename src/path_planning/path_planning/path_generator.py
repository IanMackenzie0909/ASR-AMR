#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Nav2 Fixed Goal Example (Teaching Version)

Description:
- Demonstrates how to send a fixed navigation goal to Nav2 in ROS 2
- Uses official tf_transformations to convert yaw (Euler angle) → quaternion
- Suitable for beginners to understand ROS2 Action Client and Nav2 goal sending
"""

# ==============================
# 1 Import Required ROS 2 Modules
# ==============================
import rclpy  # Core ROS 2 client library
from rclpy.node import Node  # For creating ROS 2 nodes
from rclpy.action import ActionClient  # For interacting with ROS 2 actions

from nav2_msgs.action import NavigateToPose  # Nav2 Action type
from tf_transformations import quaternion_from_euler  # Official Euler → Quaternion conversion


# ==============================
# 2 Node Class Definition
# ==============================
class Nav2FixedGoal(Node):
    """
    ROS 2 Node for sending a fixed goal to Nav2
    """
    def __init__(self):
        # Initialize the node with a name
        super().__init__('path_generator')

        # Create an Action Client to communicate with Nav2
        # - NavigateToPose: Action type used by Nav2 to move the robot
        # - '/navigate_to_pose': ROS 2 Action topic provided by Nav2
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.get_logger().info("Waiting for Nav2 action server...")

        # Wait until the Nav2 action server is ready
        # - Without this, sending goals may fail if server isn't up yet
        self._client.wait_for_server()
        self.get_logger().info("Connected to Nav2!")

    # ------------------------------
    # Method to send a fixed goal
    # ------------------------------
    def send_goal(self):
        """
        Create and send a fixed navigation goal to Nav2
        """

         # ===== Fixed target coordinates =====
        # These values can be changed to any desired goal in the map
        x = 12.8	# X position in meters
        y = 0.5	# Y position in meters
        yaw_deg = 180 	#Orientation in degrees (rotation around Z-axis)
        yaw_rad = yaw_deg * 3.14159265 / 180.0  # Convert degrees → radians

        # ===== Convert Euler angle (yaw) to quaternion =====
        # ROS 2 uses quaternions for orientations to avoid gimbal lock
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw_rad)

        # ===== Create Goal Message =====
        goal_msg = NavigateToPose.Goal()

        # Header: tells Nav2 what coordinate frame this goal is in
        goal_msg.pose.header.frame_id = 'map'  # Use map frame as reference
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()  # Timestamp for synchronization

        # Position: X, Y coordinates of the goal
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation: quaternion representing robot's heading
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        # Log info for debugging / visualization
        self.get_logger().info(
            f"Sending fixed goal → x: {x}, y: {y}, yaw: {yaw_deg} deg"
        )

        # ===== Send the goal asynchronously =====
        # - Nav2 will start moving the robot toward the goal
        self._client.send_goal_async(goal_msg)


# ==============================
# 3 Main Function
# ==============================
def main():
    """
    Main entry point for the ROS 2 node
    """
    # Initialize ROS 2 Python client library
    rclpy.init()

    # Create the node
    node = Nav2FixedGoal()

    try:
        # Send the fixed goal
        node.send_goal()

        # Keep the node alive to process callbacks (e.g., goal feedback)
        rclpy.spin(node)

    except Exception as e:
        print("Error:", e)

    finally:
        # Shutdown the node cleanly
        node.destroy_node()
        rclpy.shutdown()


# ==============================
# 4 Run main when script executed
# ==============================
if __name__ == '__main__':
    main()
