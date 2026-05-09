# #=====================================================
# # Application and Practice of Autonomous Mobile Robots
# # Copyright 2025 ASRLAB
# #=====================================================


#!/usr/bin/env python3
# coding=utf-8

"""HSV-based visual tracking node kept compatible with the tracking controller."""

import os
import rclpy
import cv2
import cv_bridge
import numpy as np
import json

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from tracking_msg.msg import Imgtracking


class VisualSensorNode(Node):
    """Detect a colored target and publish the same tracking contract as the YOLO node."""

    def __init__(self):
        """Create image subscriptions and load HSV thresholds for color tracking."""
        super().__init__('Visual_sensor')

        qos = QoSProfile(depth=10)

        self.pub = self.create_publisher(Imgtracking, 'Visual_sensor_vel', qos)

        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            qos
        )

        self.bridge = cv_bridge.CvBridge()
        self.depth_image = None

        self.prev_center = None
        self.prev_depth = None

        # HSV config
        file_path = os.path.join(
            os.getcwd(),
            'src/object_tracking/object_tracking/hsv_values.json'
        )
        with open(file_path, 'r') as f:
            self.hsv_values = json.load(f)

    # -------------------------
    # depth callback
    # -------------------------
    def depth_callback(self, msg):
        """Store the latest aligned depth frame for the RGB processing callback."""
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough'
        )

    # -------------------------
    # detection
    # -------------------------
    def image_processing(self, image):
        """Return the largest HSV-matched target contour as a bounding box."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower = (self.hsv_values[0], self.hsv_values[1], self.hsv_values[2])
        upper = (self.hsv_values[3], self.hsv_values[4], self.hsv_values[5])

        mask = cv2.inRange(hsv, lower, upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        best = None
        max_area = 0

        for c in contours:
            area = cv2.contourArea(c)
            if area < 500:
                continue
            if area > max_area:
                max_area = area
                best = cv2.boundingRect(c)

        if best is None:
            return None

        x, y, w, h = best
        cx = x + w // 2
        cy = y + h // 2

        return cx, cy, (x, y, w, h)

    # -------------------------
    # main callback
    # -------------------------
    def image_callback(self, msg):
        """Convert an RGB frame into target error and publish valid tracking state."""
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        height, width, _ = image.shape

        result = self.image_processing(image)

        # =========================
        # NO OBJECT → still show UI
        # =========================
        if result is None:
            msg_out = Imgtracking()
            msg_out.target_visible = False
            msg_out.yaw = 0.0
            msg_out.depth = -1.0
            msg_out.confidence = 0.0
            msg_out.class_id = -1
            self.pub.publish(msg_out)
            self.prev_center = None
            self.prev_depth = None
            self.draw_ui(image, width, height, None, None, -1.0)
            return

        cx, cy, box = result

        # -------------------------
        # depth (safe)
        # -------------------------
        depth = -1.0

        if self.depth_image is not None:
            h, w = self.depth_image.shape

            if 0 <= cx < w and 0 <= cy < h:
                x1 = max(cx - 2, 0)
                x2 = min(cx + 3, w)
                y1 = max(cy - 2, 0)
                y2 = min(cy + 3, h)

                region = self.depth_image[y1:y2, x1:x2]

                if region.size > 0:
                    d = np.median(region)

                    if self.depth_image.dtype == np.uint16:
                        d = d / 1000.0

                    if 0 < d < 10:
                        depth = d

        # -------------------------
        # smoothing
        # -------------------------
        if self.prev_center is None:
            smooth_cx = cx
        else:
            smooth_cx = 0.7 * self.prev_center + 0.3 * cx

        if self.prev_depth is None:
            smooth_depth = depth
        else:
            smooth_depth = 0.7 * self.prev_depth + 0.3 * depth

        self.prev_center = smooth_cx
        self.prev_depth = smooth_depth

        # -------------------------
        # error (-1 ~ 1)
        # -------------------------
        error = (width / 2 - smooth_cx) / (width / 2)
        error = max(min(error, 1.0), -1.0)

        # -------------------------
        # publish
        # -------------------------
        msg_out = Imgtracking()
        msg_out.target_visible = smooth_depth > 0
        msg_out.yaw = float(error)
        msg_out.depth = float(smooth_depth)
        msg_out.confidence = 1.0
        msg_out.class_id = 0

        self.pub.publish(msg_out)

        # -------------------------
        # UI DRAW
        # -------------------------
        self.draw_ui(image, width, height, cx, cy, smooth_depth, box, error)

    # -------------------------
    # PRO UI FUNCTION
    # -------------------------
    def draw_ui(self, image, width, height, cx, cy, depth, box=None, error=0.0):
        """Draw the color-tracking overlay used during local visual debugging."""

        center_x = width // 2
        center_y = height // 2

        # box
        if box is not None:
            x, y, w, h = box
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # target point
        if cx is not None and cy is not None:
            cv2.circle(image, (int(cx), int(cy)), 6, (0, 0, 255), -1)
            cv2.line(image, (int(cx), int(cy)), (center_x, center_y), (0, 0, 255), 2)

        # crosshair
        cv2.line(image, (center_x, 0), (center_x, height), (255, 255, 0), 2)
        cv2.line(image, (0, center_y), (width, center_y), (255, 255, 0), 2)

        # HUD panel
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (360, 150), (0, 0, 0), -1)
        image = cv2.addWeighted(overlay, 0.5, image, 0.5, 0)

        # text
        cv2.putText(image, "ASR Visual Tracker", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.putText(image, f"Error: {error:.3f}", (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.putText(image, f"Depth: {depth:.2f} m", (20, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)

        status = "TRACKING" if cx is not None else "NO TARGET"
        color = (0, 255, 0) if cx is not None else (0, 0, 255)

        cv2.putText(image, status, (20, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv2.imshow("ASR PRO VIEW", image)
        cv2.waitKey(1)


def main():
    """Run the color visual sensor node until shutdown."""
    rclpy.init()
    node = VisualSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
