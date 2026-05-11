#=====================================================
# Application and Practice of Autonomous Mobile Robots
# Copyright 2025 ASRLAB
#=====================================================

#!/usr/bin/env python3
# coding=utf-8

"""Visual servo controller that converts tracking messages into cmd_vel."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from tracking_msg.msg import Imgtracking
import time


# -------------------------
# PID parameters
# -------------------------
Kp_yaw = 1.0
Ki_yaw = 0.001
Kd_yaw = 0.05 

Kp_depth = 0.7
Ki_depth = 0.001
Kd_depth = 0.02

target_depth = 0.7


# -------------------------
# state
# -------------------------
pe_yaw = 0.0
te_yaw = 0.0

pe_depth = 0.0
te_depth = 0.0

e_yaw = 0.0
e_depth = -1.0
target_visible = False
target_confidence = 0.0
target_class_id = -1

last_time = time.time()


# -------------------------
# callback
# -------------------------
def sensor_callback(msg):
    """Update the latest visual tracking state from the detector message."""
    global e_yaw, e_depth, target_visible, target_confidence, target_class_id, last_time

    e_yaw = msg.yaw
    e_depth = msg.depth
    target_visible = bool(getattr(msg, 'target_visible', e_depth > 0))
    target_confidence = float(getattr(msg, 'confidence', 0.0))
    target_class_id = int(getattr(msg, 'class_id', -1))
    last_time = time.time()


# -------------------------
# PID
# -------------------------
def PID(P, I, D, e, pe, te):
    """Compute a compact PID output using the stored previous and integral errors."""
    te = te + e
    u = P * e + I * te + D * (e - pe)
    return u, te


# -------------------------
# clamp
# -------------------------
def clamp(x, max_v):
    """Limit a signed velocity command to the configured absolute maximum."""
    return max(min(x, max_v), -max_v)


def reset_pid_state():
    """Clear PID memory so stale errors do not move the robot after reacquiring a target."""
    global pe_yaw, te_yaw, pe_depth, te_depth

    pe_yaw = 0.0
    te_yaw = 0.0
    pe_depth = 0.0
    te_depth = 0.0


# -------------------------
# node
# -------------------------
class TaskNode(Node):
    """Convert visual tracking errors into safe Twist commands for the robot base."""

    def __init__(self):
        """Create the tracking subscriber, cmd_vel publisher, and control-loop timer."""
        super().__init__('task_node')

        qos = QoSProfile(depth=10)

        self.pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.create_subscription(
            Imgtracking,
            'Visual_sensor_vel',
            sensor_callback,
            qos
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        # 控制 log 頻率
        self.last_print_time = time.time()

    # -------------------------
    # safety check
    # -------------------------
    def sensor_ok(self):
        """Return True only when recent detector data contains a valid target and depth."""
        global e_depth, target_visible, last_time

        now = time.time()

        if now - last_time > 0.5:
            return False

        if not target_visible:
            return False

        if e_depth <= 0:
            return False

        return True

    # -------------------------
    # stop robot
    # -------------------------
    def stop(self):
        """Publish a zero Twist command to stop the robot immediately."""
        twist = Twist()
        self.pub.publish(twist)

    # -------------------------
    # debug print
    # -------------------------
    def debug_print(self, msg):
        """Throttle status logging so the control loop stays readable."""
        now = time.time()
        if now - self.last_print_time > 0.2:  # 5 Hz print
            self.get_logger().info(msg)
            self.last_print_time = now

    # -------------------------
    # control loop
    # -------------------------
    def control_loop(self):
        """Run one tracking control step or stop safely when tracking data is invalid."""
        global pe_yaw, te_yaw, pe_depth, te_depth
        global e_yaw, e_depth, target_confidence, target_class_id

        twist = Twist()

        # =========================
        # FAIL SAFE
        # =========================
        if not self.sensor_ok():
            reset_pid_state()
            self.debug_print("TRACKING LOST OR INVALID DEPTH -> STOP")
            self.stop()
            return

        # =========================
        # YAW control
        # =========================
        yaw_u, te_yaw = PID(Kp_yaw, Ki_yaw, Kd_yaw,
                            e_yaw, pe_yaw, te_yaw)
        pe_yaw = e_yaw

        # =========================
        # DEPTH control
        # =========================
        depth_error = -(target_depth - e_depth)

        depth_u, te_depth = PID(Kp_depth, Ki_depth, Kd_depth,
                                depth_error, pe_depth, te_depth)
        pe_depth = depth_error

        """ 
        safety check (when OBJECT's depth is less than 0.35m) 
        to prevent the robot from crashing into the target. 
        """
        if e_depth < 0.35:
            self.debug_print("TOO CLOSE -> STOP")
            self.stop()
            return
        
        if e_depth < 0.5:
            depth_u = - 0.1

        # =========================
        # OUTPUT
        # =========================
        twist.linear.x = clamp(depth_u, 0.5)    # forward/backward speed limited to 0.5 m/s
        twist.angular.z = clamp(yaw_u, 1.5)

        self.pub.publish(twist)

        # =========================
        # DEBUG PRINT
        # =========================
        self.debug_print(
            f"TRACKING | class={target_class_id} | conf={target_confidence:.2f} | "
            f"depth={e_depth:.2f}m | err={e_yaw:.3f} | "
            f"vx={twist.linear.x:.2f} | wz={twist.angular.z:.2f}"
        )


def main():
    """Run the visual tracking task controller until shutdown."""
    rclpy.init()
    node = TaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
