#!/usr/bin/env python
# coding=utf-8

# Import necessary modules for ROS 2 communication
import os  # Operating system module
import select  # I/O multiplexing module
import sys  # System-specific parameters and functions module
import rclpy  # ROS 2 client library

# Import message types and Quality of Service (QoS) profile for ROS 2
from geometry_msgs.msg import Twist  # ROS 2 message type for robot velocity
from rclpy.qos import QoSProfile  # Quality of Service profile for ROS 2

# Import modules for controlling terminal I/O settings
import tty  # Terminal control module


def get_key():
    """
    Function to get a key press from the user.
    
    Returns:
        str: Key pressed by the user.

    Explanation:
        - tty.setraw(sys.stdin.fileno()): Set the terminal to raw mode, disabling line buffering.
        - select.select([sys.stdin], [], [], 0.1): Monitor the standard input (keyboard) for any activity
          with a timeout of 0.1 seconds.
        - if rlist: Check if there's any activity on the standard input.
        - key = sys.stdin.read(1): Read a single character from the standard input.
        - return key: Return the key pressed by the user.
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    return key


def main():
    """
    Main function to initialize ROS 2, create a node, and publish Twist messages.

    Explanation:
        - rclpy.init(): Initialize the ROS 2 client library.
        - qos = QoSProfile(depth=10): Define a Quality of Service profile with a queue depth of 10.
          This controls how messages are queued in the system.
        - node = rclpy.create_node('wheeltec_keyboard'): Create a ROS 2 node with the name 'wheeltec_keyboard'.
          A node is a process that performs computation.
        - pub = node.create_publisher(Twist, 'cmd_vel', qos): Create a publisher that publishes
          messages of type Twist to the topic 'cmd_vel'. The publisher will use the specified QoS profile.
        - Twist: This is a message type in ROS 2 used to represent velocity commands.
          It consists of linear and angular velocity components.
        - twist: This variable holds an instance of the Twist message type, initialized with default values.
          By default, linear and angular velocities are set to zero.

    Returns:
        None
    """
    
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('robot_keyboard_control')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)
    
    try:
        while(1):
            key = get_key()
            twist = Twist() 
            if (key == 'w'):
                twist.linear.x = 0.1; twist.linear.y = 0.0;twist.linear.z = 0.0
                twist.angular.x = 0.0;twist.angular.y = 0.0;twist.angular.z = 0.0
            elif (key == 's'):
                twist.linear.x = -0.1;twist.linear.y = 0.0;twist.linear.z = 0.0
                twist.angular.x = 0.0;twist.angular.y = 0.0;twist.angular.z = 0.0
            elif (key == 'a'):
                twist.linear.x = 0.0;twist.linear.y = -0.1;twist.linear.z = 0.0
                twist.angular.x = 0.0;twist.angular.y = 0.0;twist.angular.z = 0.0
            elif (key == 'd'):
                twist.linear.x = 0.0;twist.linear.y = 0.1;twist.linear.z = 0.0
                twist.angular.x = 0.0;twist.angular.y = 0.0;twist.angular.z = 0.0
            elif (key == '\x03'):
                break
            else :
                twist.linear.x = 0.0;twist.linear.y = 0.0;twist.linear.z = 0.0
                twist.angular.x = 0.0;twist.angular.y = 0.0;twist.angular.z = 0.0
            pub.publish(twist)
            
    except Exception as e:
        print('error!!')

if __name__ == '__main__':
    main()
