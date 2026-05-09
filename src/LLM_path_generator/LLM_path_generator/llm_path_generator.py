#!/usr/bin/env python3
import sys
import os
import time
import json
import re
import yaml
import cv2
import numpy as np
import math

from PIL import Image
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler

from google import genai
from google.genai import types

# ===============================
# Global variables
# ===============================
global node_tf
global node_path
global resolution
global map_x
global map_y

# ===============================
# Google Gemini client
# ===============================
client = genai.Client(api_key="YOUR_API_KEY")

# ===============================
# Utilities
# ===============================
def pgm_to_png(pgm_path, png_path=None):
    if png_path is None:
        png_path = os.path.splitext(pgm_path)[0] + ".png"
    try:
        img = Image.open(pgm_path)
        img.save(png_path)
        print(f"PGM -> PNG success: {pgm_path} -> {png_path}")
    except Exception as e:
        print(f"PGM -> PNG failed: {e}")

def generate_waypoints(prompt_text, image_path):
    contents = [prompt_text]
    with open(image_path, "rb") as f:
        img = f.read()
    image_part = types.Part.from_bytes(data=img, mime_type="image/png")
    contents.append(image_part)

    response = client.models.generate_content(
        model="gemini-3-flash-preview",
        contents=contents,
        config=types.GenerateContentConfig(
            temperature=0.2,
            response_mime_type="application/json"
        )
    )
    return response.text

def clean_json_string(s):
    s = re.sub(r"```json", "", s)
    s = re.sub(r"```", "", s)
    return s.strip()

def parse_waypoints(s):
    s = clean_json_string(s)
    try:
        return json.loads(s)
    except Exception:
        print("Parse error:")
        print(s)
        return []

# ===============================
# ROS Nodes
# ===============================
class TFPositionReader(Node):
    def __init__(self):
        super().__init__("tf_position_reader")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.position = None

    def get_robot_position(self, timeout=5.0):
        start_time = time.time()
        while rclpy.ok():
            try:
                trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
                self.position = {
                    "x": trans.transform.translation.x,
                    "y": trans.transform.translation.y,
                    "z": trans.transform.translation.z,
                    "qx": trans.transform.rotation.x,
                    "qy": trans.transform.rotation.y,
                    "qz": trans.transform.rotation.z,
                    "qw": trans.transform.rotation.w
                }
                return self.position
            except Exception:
                if time.time() - start_time > timeout:
                    return None
                rclpy.spin_once(self, timeout_sec=0.1)

class WaypointActionPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_action_publisher')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.waypoints = []

        self.get_logger().info("Waiting for FollowWaypoints action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server ready.")

    def create_pose(self, x, y, frame='map'):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # 初始化
        return pose

    def send_waypoints(self):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Waypoints goal rejected")
            return

        self.get_logger().info("Waypoints goal accepted, robot moving...")
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info("Navigation finished successfully!")
        else:
            self.get_logger().warn(f"Navigation finished with status {result.status}")

# ===============================
# GUI Classes
# ===============================
class MapWidget(QLabel):
    mouseMoved = pyqtSignal(int, int)

    def __init__(self, image_path):
        super().__init__()
        self.cv_img = cv2.imread(image_path)
        if self.cv_img is None:
            raise FileNotFoundError(f"Cannot load map image: {image_path}")
        self.draw_img = self.cv_img.copy()
        self.scale = 1.0
        self.setMinimumSize(400, 400)
        self.setMouseTracking(True)
        self.update_qt_image()

    def resizeEvent(self, event):
        self.update_qt_image()

    def update_qt_image(self):
        rgb = cv2.cvtColor(self.draw_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        label_w, label_h = self.width(), self.height()
        self.scale = min(label_w / w, label_h / h)
        resized = cv2.resize(rgb, (int(w * self.scale), int(h * self.scale)))
        qt_img = QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1] * ch, QImage.Format_RGB888)
        self.setPixmap(QPixmap.fromImage(qt_img))

    def mouseMoveEvent(self, event):
        x = int(event.x() / self.scale)
        y = int(event.y() / self.scale)
        self.mouseMoved.emit(x, y)

    def draw_waypoints(self, waypoints):
        self.draw_img = self.cv_img.copy()
        if len(waypoints) == 0:
            self.update_qt_image()
            return
        pts = []
        for p in waypoints:
            x, y = int(p["x"]), int(p["y"])
            pts.append([x, y])
            cv2.circle(self.draw_img, (x, y), 1, (0, 0, 255), -1)   #6
        cv2.polylines(self.draw_img, [np.array(pts, np.int32)], False, (0, 0, 255), 1)  #3
        self.update_qt_image()

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LLM Navigation UI")
        self.setStyleSheet("background-color: #F0F0F0;")
        self.image_path = "/home/robot/Robot/install/wheeltec_nav2/share/wheeltec_nav2/map/map.png"

        img = cv2.imread(self.image_path)
        if img is None:
            raise FileNotFoundError(f"Map image not found: {self.image_path}")
        self.map_height, self.map_width = img.shape[:2]

        # Map Widget
        self.map = MapWidget(self.image_path)

        # Info Widgets
        self.pixel_label = QLabel("Pixel: ")
        self.pixel_label.setStyleSheet("font-size: 16px; color: darkblue;")

        self.status_label = QLabel("Status: ")
        self.status_label.setStyleSheet("font-size: 16px; color: green; font-weight: bold;")
        self.status_label.setAlignment(Qt.AlignCenter)

        self.textbox = QTextEdit()
        self.textbox.setPlaceholderText("Enter navigation request here...")
        self.textbox.setStyleSheet("font-size: 14px;")

        self.btn_generate = QPushButton("Generate Waypoints")
        self.btn_generate.setStyleSheet("""
            font-size: 16px; padding: 12px; background-color: #4CAF50; color: white; border-radius: 8px;
        """)

        self.btn_nav = QPushButton("Start Navigation")
        self.btn_nav.setStyleSheet("""
            font-size: 16px; padding: 12px; background-color: #2196F3; color: white; border-radius: 8px;
        """)

        # Layouts
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.pixel_label)
        right_layout.addWidget(self.status_label)
        right_layout.addWidget(self.textbox)
        right_layout.addWidget(self.btn_generate)
        right_layout.addWidget(self.btn_nav)
        right_layout.addStretch()

        layout = QHBoxLayout()
        layout.addWidget(self.map, 3)
        layout.addLayout(right_layout, 1)
        self.setLayout(layout)

        # Signals
        self.map.mouseMoved.connect(self.update_pixel)
        self.btn_generate.clicked.connect(self.on_generate)
        self.btn_nav.clicked.connect(self.on_navigation)
        self.waypoints = []

    def update_pixel(self, x, y):
        self.pixel_label.setText(f"Pixel: {x}, {y}")

    def set_status(self, text, color="green"):
        self.status_label.setText(f"Status: {text}")
        self.status_label.setStyleSheet(f"font-size: 16px; color: {color}; font-weight: bold;")
        QApplication.processEvents()  # 更新界面

    def on_generate(self):
        global node_tf, resolution, map_x, map_y
        self.set_status("Gemini generating...", color="orange")

        pos = node_tf.get_robot_position(timeout=5.0)
        if pos:
            pixel_x = int((pos['x'] - map_x) / resolution)
            pixel_y = int(self.map_height - (pos['y'] - map_y) / resolution)
            initial_pos_text = f"Initial pixel: x={pixel_x}, y={pixel_y}"
        else:
            initial_pos_text = "Initial pixel: unavailable"

        user_text = self.textbox.toPlainText()
        prompt = f"""
You are robot navigation planner
Rules:
- Waypoints must avoid obstacles
- Only walk on white area (255)
- Never enter gray area (205)
- Avoid black obstacles (0)
- Shortest and smooth path
- Keep 3px distance from walls
- Output JSON only [{"{"}"x":0,"y":0{"}"}]
- Pixel coordinate, origin top-left
{initial_pos_text}
User request:
{user_text}
"""
        result = generate_waypoints(prompt, self.image_path)
        self.waypoints = parse_waypoints(result)
        self.map.draw_waypoints(self.waypoints)
        self.set_status("Waypoints generated", color="green")

    def on_navigation(self):
        global node_path, resolution, map_x, map_y
        self.set_status("Navigating...", color="blue")

        node_path.waypoints = []
        num_wp = len(self.waypoints)
        for i, p in enumerate(self.waypoints):
            pixel_x, pixel_y = int(p["x"]), int(p["y"])
            map_x_world = pixel_x * resolution + map_x
            map_y_world = (self.map_height - pixel_y) * resolution + map_y

            pose = node_path.create_pose(map_x_world, map_y_world)

            # 計算方向
            if i < num_wp - 1:
                next_p = self.waypoints[i + 1]
                next_x = next_p["x"] * resolution + map_x
                next_y = (self.map_height - next_p["y"]) * resolution + map_y
                dx = next_x - map_x_world
                dy = next_y - map_y_world
                yaw = math.atan2(dy, dx)
            elif i > 0:
                prev_p = self.waypoints[i - 1]
                prev_x = prev_p["x"] * resolution + map_x
                prev_y = (self.map_height - prev_p["y"]) * resolution + map_y
                dx = map_x_world - prev_x
                dy = map_y_world - prev_y
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0  # 單 waypoint

            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            node_path.waypoints.append(pose)

        node_path.send_waypoints()
        self.set_status("Navigation finished", color="green")

# ===============================
# Main
# ===============================
def main(args=None):
    # Convert map
    map_path = "/home/robot/Robot/install/wheeltec_nav2/share/wheeltec_nav2/map/WHEELTEC.pgm"
    yaml_path = "/home/robot/Robot/install/wheeltec_nav2/share/wheeltec_nav2/map/WHEELTEC.yaml"
    png_path = "/home/robot/Robot/install/wheeltec_nav2/share/wheeltec_nav2/map/map.png"
    pgm_to_png(map_path, png_path)

    global resolution, map_x, map_y, node_tf, node_path
    with open(yaml_path, "r") as f:
        map_info = yaml.safe_load(f)
    resolution = map_info["resolution"]
    origin = map_info["origin"]
    map_x = float(origin[0])
    map_y = float(origin[1])

    # Initialize ROS
    rclpy.init(args=args)
    node_tf = TFPositionReader()
    node_path = WaypointActionPublisher()

    # Start Qt app
    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(1400, 800)
    window.show()
    sys.exit(app.exec_())

    node_tf.destroy_node()
    node_path.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
