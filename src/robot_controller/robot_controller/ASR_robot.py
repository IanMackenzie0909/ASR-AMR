# #!/usr/bin/env python
# # coding=utf-8

# # Import necessary modules for ROS 2 communication
# import os  # Operating system module
# import select  # I/O multiplexing module
# import sys  # System-specific parameters and functions module
# import rclpy  # ROS 2 client library

# # Import message types and Quality of Service (QoS) profile for ROS 2
# from geometry_msgs.msg import Twist  # ROS 2 message type for robot velocity
# from rclpy.qos import QoSProfile  # Quality of Service profile for ROS 2
# from nav_msgs.msg import Odometry

# # Import modules for controlling terminal I/O settings
# import tty  # Terminal control module
# import serial
# import struct

# ser = serial.Serial('/dev/ttyTHS1',115200,timeout=1)


# def send_command(command_hex):
#     if ser and ser.is_open:
#         ser.write(command_hex)
#     else:
#         print("Serial port not available.")

# def assemble_packet(vx, vy, vz):
#     packet = bytearray.fromhex('3E 01 09')
#     packet.extend(vx)
#     packet.extend(vy)
#     packet.extend(vz)
#     return packet

# def receive_packet():
#     if ser.in_waiting >= 15:   # check if a full packet is available
#         data = ser.read(15)

#         if data[0] == 0x3E:   # check header
#             odomx = struct.unpack('>i', data[3:7])[0] / 100.0
#             odomy = struct.unpack('>i', data[7:11])[0] / 100.0
#             odomz = struct.unpack('>i', data[11:15])[0] / 100.0



#             print(f"Feedback -> vx:{odomx}, vy:{odomy}, vz:{odomz}")

# def cmd_callback(msg):
#         vx = msg.linear.x
#         vy = msg.linear.y
#         vz = msg.angular.z
        
#         x = struct.pack('>h', int(msg.linear.x * 100))
#         y = struct.pack('>h', int(msg.linear.y * 100))
#         z = struct.pack('>h', int(msg.angular.z * 100))

#         packet = assemble_packet(x, y, z)
#         send_command(packet)

#         print(f"Received -> vx:{vx}, vy:{vy}, vz:{vz}")

# def timer_callback():

#     packet = bytearray.fromhex('3E 02 03')
#     if ser and ser.is_open:
#         ser.write(packet)

#     receive_packet()

# def main():
#     print('Hi from robot_controller.')
    
#     rclpy.init()
#     qos = QoSProfile(depth=10)
#     node = rclpy.create_node('ASR_Robot_Controller')
#     sub = node.create_subscription(Twist, 'cmd_vel', cmd_callback, qos)

#     odom_pub = node.create_publisher(Odometry, 'odom', qos)

#     timer = node.create_timer(0.25, timer_callback)

#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
    
# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import serial
import struct
from tf_transformations import quaternion_from_euler

ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=1)

class ASRRobotController(Node):

    def __init__(self):
        super().__init__('ASR_Robot_Controller')

        # Subscriber to /cmd_vel
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10
        )

        # Publisher for /odom
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Publish to /imu/data
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)

        # Timer to send odom request every 0.25s
        self.timer = self.create_timer(0.25, self.timer_callback)

    def cmd_callback(self, msg):
        # Convert velocities to bytes
        vx = struct.pack('>h', int(msg.linear.x * 100))
        vy = struct.pack('>h', int(msg.linear.y * 100))
        vz = struct.pack('>h', int(msg.angular.z * 100))

        packet = self.assemble_packet(vx, vy, vz)
        self.send_command(packet)

        # self.get_logger().info(f"Received cmd_vel -> vx:{msg.linear.x}, vy:{msg.linear.y}, vz:{msg.angular.z}")

    def assemble_packet(self, vx, vy, vz):
        packet = bytearray.fromhex('3E 01 09')
        packet.extend(vx)
        packet.extend(vy)
        packet.extend(vz)
        return packet

    def send_command(self, packet):
        if ser and ser.is_open:
            ser.write(packet)

    def receive_packet(self):
        if ser.in_waiting >= 53:
            data = ser.read(53)
            if data[0] == 0x3E:
                # Parse int32 odom values
                velx = struct.unpack('>i', data[3:7])[0] / 100.0
                vely = struct.unpack('>i', data[7:11])[0] / 100.0
                velz = struct.unpack('>i', data[11:15])[0] / 100.0

                odomx = struct.unpack('>i', data[15:19])[0] / 100.0
                odomy = struct.unpack('>i', data[19:23])[0] / 100.0
                odomz = struct.unpack('>i', data[23:27])[0] / 100.0

                imu_accel_x = struct.unpack('>h', data[27:29])[0] / 100.0
                imu_accel_y = struct.unpack('>h', data[29:31])[0] / 100.0
                imu_accel_z = struct.unpack('>h', data[31:33])[0] / 100.0

                imu_veloc_x = struct.unpack('>h', data[33:35])[0] / 100.0 / 180.0 * 3.14159
                imu_veloc_y = struct.unpack('>h', data[35:37])[0] / 100.0 / 180.0 * 3.14159
                imu_veloc_z = struct.unpack('>h', data[37:39])[0] / 100.0 / 180.0 * 3.14159

                imu_angle_x = struct.unpack('>h', data[39:41])[0] / 100.0 / 180.0 * 3.14159
                imu_angle_y = struct.unpack('>h', data[41:43])[0] / 100.0 / 180.0 * 3.14159
                imu_angle_z = struct.unpack('>h', data[43:45])[0] / 100.0 / 180.0 * 3.14159

                imu_quate_1 = struct.unpack('>h', data[45:47])[0] / 10000.0
                imu_quate_2 = struct.unpack('>h', data[47:49])[0] / 10000.0
                imu_quate_3 = struct.unpack('>h', data[49:51])[0] / 10000.0
                imu_quate_4 = struct.unpack('>h', data[51:53])[0] / 10000.0

                # Publish to /odom
                msg = Odometry()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'odom'
                # msg.child_frame_id.frame_id = 'base_link'
                msg.pose.pose.position.x = odomx
                msg.pose.pose.position.y = odomy
                qx, qy, qz, qw = quaternion_from_euler(0, 0, odomz)
                msg.pose.pose.orientation.x = qx
                msg.pose.pose.orientation.y = qy
                msg.pose.pose.orientation.z = qz
                msg.pose.pose.orientation.w = qw
                msg.twist.twist.linear.x = velx
                msg.twist.twist.linear.y = vely
                msg.twist.twist.angular.z = velz/32.0*36.0
                self.odom_pub.publish(msg)
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'gyro_link'
                # Orientation quaternion (x, y, z, w)
                msg.orientation.x = imu_quate_2
                msg.orientation.y = imu_quate_3
                msg.orientation.z = imu_quate_4
                msg.orientation.w = imu_quate_1
                # Angular velocity (rad/s)
                msg.angular_velocity.x = imu_veloc_x
                msg.angular_velocity.y = imu_veloc_y
                msg.angular_velocity.z = imu_veloc_z
                # Linear acceleration (m/s^2)
                msg.linear_acceleration.x = imu_accel_x
                msg.linear_acceleration.y = imu_accel_y
                msg.linear_acceleration.z = imu_accel_z  # gravity
                # Publish message
                self.imu_pub.publish(msg)
                # self.get_logger().info("IMU message published")

                # self.get_logger().info(f"Odom -> vx:{odomx}, vy:{odomy}, vz:{odomz}")
                self.get_logger().info(f"Odom -> x:{odomx}, y:{imu_quate_3}, z:{imu_quate_4}")
                # self.get_logger().info(f"check -> 1:{velz/3.14*180}, 2:{imu_veloc_z/3.14*180}, z:{odomz/3.14*180}")

    def timer_callback(self):
        # # Send odom request packet
        # packet = bytearray.fromhex('3E 02 03')
        # if ser and ser.is_open:
        #     ser.write(packet)

        # # Receive response from robot
        # self.receive_packet()

        # Send request packet
        packet = bytearray.fromhex('3E 04 03')
        if ser and ser.is_open:
            ser.write(packet)

        # Receive response from robot
        self.receive_packet()


def main():
    rclpy.init()
    node = ASRRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()