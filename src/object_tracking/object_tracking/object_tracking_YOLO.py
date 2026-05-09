#=====================================================
# Application and Practice of Autonomous Mobile Robots
# Copyright 2025 ASRLAB
#=====================================================

#!/usr/bin/env python3
# coding=utf-8

"""YOLO-based visual tracking node for publishing safe target state."""

import rclpy
import cv2
import cv_bridge
import numpy as np
from pathlib import Path

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from tracking_msg.msg import Imgtracking

from ultralytics import YOLO


class VisualSensorNode(Node):
    """Detect a target with YOLO and publish controller-ready tracking state."""

    def __init__(self):
        """Create camera subscriptions, load the YOLO model, and prepare smoothing state."""
        super().__init__('Visual_sensor')

        qos = QoSProfile(depth=10)

        # -------------------------
        # publisher
        # -------------------------
        self.pub = self.create_publisher(
            Imgtracking,
            'Visual_sensor_vel',
            qos
        )

        # -------------------------
        # RGB image subscriber
        # -------------------------
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos
        )

        # -------------------------
        # aligned depth subscriber
        # -------------------------
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            qos
        )

        self.bridge = cv_bridge.CvBridge()
        self.depth_image = None

        # smoothing memory
        self.prev_center = None
        self.prev_depth = None

        # -------------------------
        # YOLO model
        # -------------------------
        default_model_path = str(Path(__file__).with_name("best.pt"))
        self.declare_parameter("model_path", default_model_path)
        self.declare_parameter("confidence_threshold", 0.75)
        self.model_path = self.get_parameter("model_path").value
        self.conf_thres = float(self.get_parameter("confidence_threshold").value)

        self.model = YOLO(self.model_path)

        # =====================================================
        # Important:
        # Only keep detection boxes whose confidence >= 0.75
        # If more than one object is detected, keep only the highest confidence one
        # =====================================================
        self.get_logger().info("YOLO model loaded successfully.")
        self.get_logger().info(f"YOLO model path: {self.model_path}")
        self.get_logger().info("Visual Sensor Node started.")
        self.get_logger().info(
            f"Detection rule: keep only highest confidence object, conf >= {self.conf_thres:.2f}"
        )

    # -------------------------
    # depth callback
    # -------------------------
    def depth_callback(self, msg):
        """Store the latest aligned depth frame for RGB callback processing."""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )
        except Exception as e:
            self.get_logger().error(f"Depth image conversion failed: {e}")

    # -------------------------
    # YOLO detection
    # -------------------------
    def image_processing(self, image):
        """Return the highest-confidence detection that passes the configured threshold."""

        try:
            results = self.model(
                image,
                conf=self.conf_thres,
                verbose=False
            )
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return None

        if len(results) == 0:
            return None

        result = results[0]

        if result.boxes is None or len(result.boxes) == 0:
            return None

        best_box = None
        best_conf = -1.0
        best_cls = -1

        valid_count = 0

        for box in result.boxes:
            xyxy = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0].cpu().numpy())
            cls_id = int(box.cls[0].cpu().numpy())

            # =====================================================
            # Delete detection box if confidence is lower than threshold
            # =====================================================
            if conf < self.conf_thres:
                continue

            valid_count += 1

            x1, y1, x2, y2 = xyxy

            w = x2 - x1
            h = y2 - y1

            # =====================================================
            # If there are multiple objects,
            # keep only the one with highest confidence
            # =====================================================
            if conf > best_conf:
                best_conf = conf
                best_cls = cls_id
                best_box = (
                    int(x1),
                    int(y1),
                    int(w),
                    int(h)
                )

        # No object confidence is higher than threshold
        if best_box is None:
            return None

        if valid_count > 1:
            self.get_logger().warn(
                f"Multiple objects detected: {valid_count}. "
                f"Keep highest confidence one: conf={best_conf:.2f}"
            )

        x, y, w, h = best_box

        cx = x + w // 2
        cy = y + h // 2

        return cx, cy, best_box, best_cls, best_conf

    def reset_smoothing(self):
        """Clear temporal smoothing when the target disappears or depth becomes invalid."""
        self.prev_center = None
        self.prev_depth = None

    def publish_tracking(
        self,
        target_visible,
        yaw=0.0,
        depth=-1.0,
        confidence=0.0,
        class_id=-1
    ):
        """Publish one tracking message with explicit target validity metadata."""
        msg_out = Imgtracking()
        msg_out.target_visible = bool(target_visible)
        msg_out.yaw = float(yaw)
        msg_out.depth = float(depth)
        msg_out.confidence = float(confidence)
        msg_out.class_id = int(class_id)
        self.pub.publish(msg_out)

    def estimate_depth(self, box):
        """Estimate target distance from the median valid depth inside the box center area."""
        if self.depth_image is None:
            return -1.0

        img_h, img_w = self.depth_image.shape[:2]
        x, y, box_w, box_h = box

        x1 = max(int(x + box_w * 0.25), 0)
        x2 = min(int(x + box_w * 0.75), img_w)
        y1 = max(int(y + box_h * 0.25), 0)
        y2 = min(int(y + box_h * 0.75), img_h)

        if x1 >= x2 or y1 >= y2:
            return -1.0

        region = self.depth_image[y1:y2, x1:x2]
        valid_region = region[region > 0]

        if valid_region.size == 0:
            return -1.0

        depth = np.median(valid_region)
        if self.depth_image.dtype == np.uint16:
            depth = depth / 1000.0

        if 0 < depth < 10:
            return float(depth)

        return -1.0

    # -------------------------
    # main RGB callback
    # -------------------------
    def image_callback(self, msg):
        """Convert RGB frames, run detection, estimate depth, and publish tracking state."""
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f"RGB image conversion failed: {e}")
            return

        height, width, _ = image.shape

        result = self.image_processing(image)

        # =========================
        # no object detected
        # =========================
        if result is None:
            self.publish_tracking(False)

            # Reset smoothing memory when target disappears
            self.reset_smoothing()

            self.draw_ui(
                image=image,
                width=width,
                height=height,
                cx=None,
                cy=None,
                depth=-1.0,
                box=None,
                error=0.0,
                cls_id=None,
                conf=None
            )

            return

        cx, cy, box, cls_id, conf = result

        # -------------------------
        # depth estimation
        # -------------------------
        depth = self.estimate_depth(box)


        # -------------------------
        # center smoothing
        # -------------------------
        if self.prev_center is None:
            smooth_cx = float(cx)
        else:
            smooth_cx = 0.7 * self.prev_center + 0.3 * float(cx)

        self.prev_center = smooth_cx

        # -------------------------
        # depth smoothing
        # Only update when depth is valid
        # -------------------------
        if depth > 0:
            if self.prev_depth is None:
                smooth_depth = depth
            else:
                smooth_depth = 0.7 * self.prev_depth + 0.3 * depth

            self.prev_depth = smooth_depth
        else:
            if self.prev_depth is not None:
                smooth_depth = self.prev_depth
            else:
                smooth_depth = -1.0

        if smooth_depth <= 0:
            self.publish_tracking(False, confidence=conf, class_id=cls_id)
            self.draw_ui(
                image=image,
                width=width,
                height=height,
                cx=cx,
                cy=cy,
                depth=smooth_depth,
                box=box,
                error=0.0,
                cls_id=cls_id,
                conf=conf
            )
            return

        # -------------------------
        # normalized yaw error
        # center: 0
        # object at left  -> positive
        # object at right -> negative
        # range: -1 ~ 1
        # -------------------------
        error = (width / 2 - smooth_cx) / (width / 2)
        error = max(min(error, 1.0), -1.0)

        # -------------------------
        # publish tracking result
        # -------------------------
        self.publish_tracking(
            True,
            yaw=error,
            depth=smooth_depth,
            confidence=conf,
            class_id=cls_id
        )

        # -------------------------
        # draw UI
        # -------------------------
        self.draw_ui(
            image=image,
            width=width,
            height=height,
            cx=cx,
            cy=cy,
            depth=smooth_depth,
            box=box,
            error=error,
            cls_id=cls_id,
            conf=conf
        )

    # -------------------------
    # UI drawing function
    # -------------------------
    def draw_ui(
        self,
        image,
        width,
        height,
        cx,
        cy,
        depth,
        box=None,
        error=0.0,
        cls_id=None,
        conf=None
    ):
        """Draw the detector overlay used during local visual debugging."""

        center_x = width // 2
        center_y = height // 2

        # -------------------------
        # bounding box
        # -------------------------
        if box is not None:
            x, y, w, h = box

            cv2.rectangle(
                image,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                2
            )

            label = "YOLO"

            if cls_id is not None and conf is not None:
                label = f"Shin-chan | {conf * 100:.0f}%"

            cv2.putText(
                image,
                label,
                (x, max(y - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        # -------------------------
        # target center point
        # -------------------------
        if cx is not None and cy is not None:
            cv2.circle(
                image,
                (int(cx), int(cy)),
                6,
                (0, 0, 255),
                -1
            )

            cv2.line(
                image,
                (int(cx), int(cy)),
                (center_x, center_y),
                (0, 0, 255),
                2
            )

        # -------------------------
        # image center crosshair
        # -------------------------
        cv2.line(
            image,
            (center_x, 0),
            (center_x, height),
            (255, 255, 0),
            2
        )

        cv2.line(
            image,
            (0, center_y),
            (width, center_y),
            (255, 255, 0),
            2
        )

        # -------------------------
        # HUD panel
        # -------------------------
        overlay = image.copy()

        cv2.rectangle(
            overlay,
            (10, 10),
            (430, 190),
            (0, 0, 0),
            -1
        )

        image = cv2.addWeighted(
            overlay,
            0.5,
            image,
            0.5,
            0
        )

        # -------------------------
        # HUD text
        # -------------------------
        cv2.putText(
            image,
            "ASR YOLO Visual Tracker",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.putText(
            image,
            f"Error: {error:.3f}",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2
        )

        cv2.putText(
            image,
            f"Depth: {depth:.2f} m",
            (20, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 200, 0),
            2
        )

        if cls_id is not None and conf is not None:
            cv2.putText(
                image,
                f"Class: {cls_id} | Conf: {conf:.2f}",
                (20, 130),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            cv2.putText(
                image,
                "Rule: keep highest conf, conf >= 0.75",
                (20, 160),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2
            )

        status = "TRACKING" if cx is not None else "NO TARGET"
        color = (0, 255, 0) if cx is not None else (0, 0, 255)

        cv2.putText(
            image,
            status,
            (20, 185),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2
        )

        # -------------------------
        # show image
        # -------------------------
        cv2.imshow("ASR YOLO PRO VIEW", image)
        cv2.waitKey(1)


def main():
    """Run the YOLO visual sensor node until shutdown."""
    rclpy.init()

    node = VisualSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
