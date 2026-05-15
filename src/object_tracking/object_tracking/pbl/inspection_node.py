#!/usr/bin/env python3
# coding=utf-8

"""Autonomous PBL inspection mission for four AMR vision tasks."""

import json
import math
import time
from pathlib import Path

import cv2
import cv_bridge
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tracking_msg.srv import VisionAnalyze
from tf_transformations import quaternion_from_euler

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


DEFAULT_GOALS = [
    {
        "name": "cup_water",
        "task": "water_level",
        "x": 0.0,
        "y": 0.0,
        "yaw": 0.0,
    },
    {
        "name": "multimeter",
        "task": "multimeter",
        "x": 0.0,
        "y": 0.0,
        "yaw": 0.0,
    },
    {
        "name": "tower_light",
        "task": "tower_light",
        "x": 0.0,
        "y": 0.0,
        "yaw": 0.0,
    },
    {
        "name": "baseball",
        "task": "baseball",
        "x": 0.0,
        "y": 0.0,
        "yaw": 0.0,
    },
]

MISSION_TASKS = ("water_level", "multimeter", "tower_light", "baseball")

DEFAULT_SEARCH_YAWS = [0.0, 90.0, 180.0, -90.0]


class PBLInspectionNode(Node):
    """Navigate to inspection points, analyze images, and publish results."""

    def __init__(self):
        super().__init__("pbl_inspection")

        qos = QoSProfile(depth=10)
        self.bridge = cv_bridge.CvBridge()
        self.latest_image = None
        self.latest_depth = None
        self.latest_image_time = 0.0
        self.results = []
        self.pending_tasks = set(MISSION_TASKS)
        self.completed_tasks = {}
        self.goal_index = 0
        self.started = False
        self.inspect_timer = None
        self.current_pose = None
        self.start_pose = None
        self.mission_start_time = None
        self.mission_deadline = None
        self.mission_complete = False
        self.returning_to_start = False
        self.return_reason = None

        default_model_path = str(Path(__file__).resolve().parents[1] / "best.pt")
        default_map_yaml = str(
            Path.cwd() / "src" / "wheeltec_robot_nav2" / "map" / "WHEELTEC.yaml"
        )
        self.declare_parameter("goals_json", json.dumps(DEFAULT_GOALS))
        self.declare_parameter("mission_mode", "auto")
        self.declare_parameter("mission_time_limit_sec", 175.0)
        self.declare_parameter("optimize_goal_order", False)
        self.declare_parameter("return_to_start", True)
        self.declare_parameter("map_yaml_file", default_map_yaml)
        self.declare_parameter("search_waypoints_json", "")
        self.declare_parameter(
            "search_yaw_offsets_json",
            json.dumps(DEFAULT_SEARCH_YAWS),
        )
        self.declare_parameter("search_spacing_m", 2.0)
        self.declare_parameter("search_max_waypoints", 4)
        self.declare_parameter("search_clearance_m", 0.35)
        self.declare_parameter("model_path", default_model_path)
        self.declare_parameter("confidence_threshold", 0.50)
        self.declare_parameter("save_dir", "/tmp/pbl_inspection")
        self.declare_parameter("wait_after_arrival_sec", 0.4)
        self.declare_parameter("skip_navigation", False)
        self.declare_parameter("water_roi_json", "")
        self.declare_parameter("cup_rois_json", "")
        self.declare_parameter("display_window", False)
        self.declare_parameter("vision_api_service", "/pbl_vision_api/analyze")
        self.declare_parameter("vision_api_timeout_sec", 8.0)
        self.declare_parameter("vision_api_required_tasks_json", '["water_level"]')
        self.declare_parameter("vision_api_fallback_tasks_json", '["multimeter"]')
        self.declare_parameter(
            "camera_topic",
            "/camera/camera/color/image_raw",
        )
        self.declare_parameter(
            "depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter(
            "multimeter_labels",
            ["multimeter", "multi-meter", "meter", "digital multimeter"],
        )

        self.goals = self.load_goals()
        self.model_path = self.get_parameter("model_path").value
        self.conf_thres = float(
            self.get_parameter("confidence_threshold").value
        )
        self.save_dir = Path(self.get_parameter("save_dir").value)
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.wait_after_arrival = float(
            self.get_parameter("wait_after_arrival_sec").value
        )
        self.skip_navigation = bool(
            self.get_parameter("skip_navigation").value
        )
        self.water_roi = self.load_water_roi()
        self.cup_rois = self.load_cup_rois()
        self.display_window = bool(
            self.get_parameter("display_window").value
        )
        self.multimeter_labels = {
            str(label).lower()
            for label in self.get_parameter("multimeter_labels").value
        }
        self.vision_api_timeout = float(
            self.get_parameter("vision_api_timeout_sec").value
        )
        self.vision_api_required_tasks = self.load_task_list_parameter(
            "vision_api_required_tasks_json"
        )
        self.vision_api_fallback_tasks = self.load_task_list_parameter(
            "vision_api_fallback_tasks_json"
        )

        self.yolo = self.load_yolo_model()
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )
        self.vision_api_client = self.create_client(
            VisionAnalyze,
            self.get_parameter("vision_api_service").value,
        )

        self.result_pub = self.create_publisher(
            String,
            "pbl_inspection/result",
            qos,
        )
        self.annotated_pub = self.create_publisher(
            Image,
            "pbl_inspection/annotated_image",
            qos,
        )
        self.create_subscription(
            Image,
            self.get_parameter("camera_topic").value,
            self.image_callback,
            qos,
        )
        self.create_subscription(
            Image,
            self.get_parameter("depth_topic").value,
            self.depth_callback,
            qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("amcl_pose_topic").value,
            self.pose_callback,
            qos,
        )

        self.start_timer = self.create_timer(0.5, self.start_once)

    def load_goals(self):
        """Parse the mission goal list from JSON."""
        try:
            goals = json.loads(self.get_parameter("goals_json").value)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid goals_json: {exc}")
            return DEFAULT_GOALS

        if not isinstance(goals, list) or len(goals) != 4:
            self.get_logger().warn(
                "goals_json must contain exactly four goals."
            )
            return DEFAULT_GOALS

        return goals

    def load_water_roi(self):
        """Return an optional calibrated water ROI as x, y, w, h."""
        raw = self.get_parameter("water_roi_json").value
        if not raw:
            return None

        try:
            roi = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Invalid water_roi_json: {exc}")
            return None

        if isinstance(roi, list) and len(roi) == 4:
            return tuple(int(v) for v in roi)

        self.get_logger().warn("water_roi_json must be [x, y, width, height].")
        return None

    def load_cup_rois(self):
        """Return optional calibrated cup ROIs as a list of x, y, w, h boxes."""
        raw = self.get_parameter("cup_rois_json").value
        if not raw:
            return []

        try:
            rois = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Invalid cup_rois_json: {exc}")
            return []

        if not isinstance(rois, list):
            self.get_logger().warn("cup_rois_json must be a list of boxes.")
            return []

        boxes = []
        for roi in rois:
            if isinstance(roi, list) and len(roi) == 4:
                boxes.append(tuple(int(v) for v in roi))
            else:
                self.get_logger().warn(
                    "Each cup ROI must be [x, y, width, height]."
                )
        return boxes

    def load_task_list_parameter(self, parameter_name):
        """Load a JSON task-name list parameter as a lowercase set."""
        raw = self.get_parameter(parameter_name).value
        try:
            values = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Invalid {parameter_name}: {exc}")
            return set()

        if not isinstance(values, list):
            self.get_logger().warn(f"{parameter_name} must be a JSON list.")
            return set()

        return {str(value).lower() for value in values}

    def load_yolo_model(self):
        """Load YOLO when available, otherwise use OpenCV only."""
        if YOLO is None:
            self.get_logger().warn(
                "ultralytics is not installed; YOLO is disabled."
            )
            return None

        try:
            model = YOLO(self.model_path)
        except Exception as exc:
            self.get_logger().warn(f"Could not load YOLO model: {exc}")
            return None

        self.get_logger().info(f"YOLO model loaded: {self.model_path}")
        return model

    def image_callback(self, msg):
        """Store the latest color frame."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image_time = time.time()
        except Exception as exc:
            self.get_logger().error(f"RGB conversion failed: {exc}")

    def depth_callback(self, msg):
        """Store the latest depth frame."""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )
        except Exception as exc:
            self.get_logger().warn(f"Depth conversion failed: {exc}")

    def pose_callback(self, msg):
        """Store the latest AMCL pose for route ordering."""
        pose = msg.pose.pose.position
        self.current_pose = (float(pose.x), float(pose.y))

    def start_once(self):
        """Start the mission after the first camera frame arrives."""
        if self.started:
            return

        if self.latest_image is None:
            self.get_logger().info("Waiting for camera image...")
            return

        return_to_start = bool(self.get_parameter("return_to_start").value)
        if return_to_start and not self.skip_navigation and self.current_pose is None:
            self.get_logger().info("Waiting for AMCL pose before mission start...")
            return

        self.started = True
        self.start_timer.cancel()

        if not self.skip_navigation:
            self.get_logger().info("Waiting for Nav2 action server...")
            if not self.nav_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Nav2 action server is not available.")
                self.started = False
                self.start_timer = self.create_timer(0.5, self.start_once)
                return

        self.mission_start_time = time.time()
        self.start_pose = self.current_pose
        if self.start_pose is None:
            self.get_logger().warn(
                "No AMCL pose received yet; return-to-start is unavailable."
            )
        limit = float(self.get_parameter("mission_time_limit_sec").value)
        self.mission_deadline = self.mission_start_time + max(1.0, limit)
        self.configure_mission_route()
        self.publish_result({
            "event": "mission_start",
            "mode": self.get_parameter("mission_mode").value,
            "goal_count": len(self.goals),
            "time_limit_sec": limit,
            "task_order": list(MISSION_TASKS),
            "pending_tasks": self.ordered_pending_tasks(),
            "return_to_start": bool(self.get_parameter("return_to_start").value),
        })
        self.get_logger().info("Starting PBL inspection mission.")
        self.send_next_goal()

    def configure_mission_route(self):
        """Choose a fast fixed route or a map-based search route."""
        mode = str(self.get_parameter("mission_mode").value).lower()
        if mode not in {"auto", "fixed", "search"}:
            self.get_logger().warn(f"Unknown mission_mode={mode}; using auto.")
            mode = "auto"

        fixed_points_known = any(
            abs(float(goal.get("x", 0.0))) > 1e-3
            or abs(float(goal.get("y", 0.0))) > 1e-3
            for goal in self.goals
        )
        use_search = (
            mode == "search"
            or (mode == "auto" and not fixed_points_known)
        )

        if use_search:
            self.goals = self.build_search_goals()
            self.get_logger().info(
                f"Using map/search mission route with {len(self.goals)} goals."
            )
            return

        if bool(self.get_parameter("optimize_goal_order").value):
            self.goals = self.order_goals_nearest_neighbor(self.goals)
            self.get_logger().info("Using optimized fixed inspection route.")

    def build_search_goals(self):
        """Build a time-bounded search route when inspection coordinates are unknown."""
        waypoints = self.load_configured_search_waypoints()
        if not waypoints:
            waypoints = self.generate_search_waypoints_from_map()

        if not waypoints:
            start_x, start_y = self.current_pose or (0.0, 0.0)
            waypoints = [{"x": start_x, "y": start_y, "yaw": 0.0}]
            self.get_logger().warn(
                "No search waypoints available; using in-place scan."
            )

        waypoints = self.order_goals_nearest_neighbor(waypoints)
        yaw_offsets = self.load_search_yaws()
        goals = []
        for index, waypoint in enumerate(waypoints, start=1):
            base_yaw = float(waypoint.get("yaw", 0.0))
            for yaw_offset in yaw_offsets:
                goals.append({
                    "name": f"search_{index}_{int(yaw_offset)}",
                    "task": "search",
                    "x": float(waypoint.get("x", 0.0)),
                    "y": float(waypoint.get("y", 0.0)),
                    "yaw": base_yaw + float(yaw_offset),
                })
        return goals

    def load_configured_search_waypoints(self):
        """Parse optional search waypoints from JSON."""
        raw = self.get_parameter("search_waypoints_json").value
        if not raw:
            return []

        try:
            items = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Invalid search_waypoints_json: {exc}")
            return []

        waypoints = []
        if not isinstance(items, list):
            self.get_logger().warn("search_waypoints_json must be a list.")
            return []

        for index, item in enumerate(items, start=1):
            if isinstance(item, dict):
                waypoints.append({
                    "name": item.get("name", f"search_{index}"),
                    "x": float(item.get("x", 0.0)),
                    "y": float(item.get("y", 0.0)),
                    "yaw": float(item.get("yaw", 0.0)),
                })
            elif isinstance(item, list) and len(item) >= 2:
                waypoints.append({
                    "name": f"search_{index}",
                    "x": float(item[0]),
                    "y": float(item[1]),
                    "yaw": float(item[2]) if len(item) >= 3 else 0.0,
                })
        return waypoints

    def load_search_yaws(self):
        """Parse yaw offsets used to quickly scan each search waypoint."""
        raw = self.get_parameter("search_yaw_offsets_json").value
        try:
            yaws = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Invalid search_yaw_offsets_json: {exc}")
            return DEFAULT_SEARCH_YAWS

        if not isinstance(yaws, list) or not yaws:
            return DEFAULT_SEARCH_YAWS
        return [float(yaw) for yaw in yaws]

    def generate_search_waypoints_from_map(self):
        """Generate a small set of free-space waypoints from the occupancy map."""
        map_yaml = Path(self.get_parameter("map_yaml_file").value)
        if not map_yaml.exists():
            self.get_logger().warn(f"Map yaml not found: {map_yaml}")
            return []

        try:
            map_info = self.read_simple_map_yaml(map_yaml)
            image_path = Path(map_info["image"])
            if not image_path.is_absolute():
                image_path = map_yaml.parent / image_path
            resolution = float(map_info["resolution"])
            origin = map_info["origin"]
        except (KeyError, ValueError, OSError) as exc:
            self.get_logger().warn(f"Could not read map yaml: {exc}")
            return []

        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            self.get_logger().warn(f"Could not read map image: {image_path}")
            return []

        free = np.where(image > 250, 255, 0).astype(np.uint8)
        clearance_px = max(
            1,
            int(float(self.get_parameter("search_clearance_m").value) / resolution),
        )
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (2 * clearance_px + 1, 2 * clearance_px + 1),
        )
        free = cv2.erode(free, kernel, iterations=1)

        spacing_px = max(
            1,
            int(float(self.get_parameter("search_spacing_m").value) / resolution),
        )
        height, width = free.shape
        candidates = []
        for row in range(spacing_px // 2, height, spacing_px):
            for col in range(spacing_px // 2, width, spacing_px):
                if free[row, col] == 0:
                    continue
                wx = float(origin[0]) + (col + 0.5) * resolution
                wy = float(origin[1]) + (height - row - 0.5) * resolution
                candidates.append({"x": wx, "y": wy, "yaw": 0.0})

        max_count = int(self.get_parameter("search_max_waypoints").value)
        selected = self.select_spread_waypoints(candidates, max_count)
        self.get_logger().info(
            f"Generated {len(selected)} search waypoints from {map_yaml}."
        )
        return selected

    def read_simple_map_yaml(self, path):
        """Read the small subset of map YAML fields without requiring PyYAML."""
        data = {}
        for line in path.read_text(encoding="utf-8").splitlines():
            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            key = key.strip()
            value = value.strip().strip("'\"")
            if key == "origin":
                data[key] = json.loads(value)
            elif key in {"resolution", "occupied_thresh", "free_thresh"}:
                data[key] = float(value)
            else:
                data[key] = value
        return data

    def select_spread_waypoints(self, candidates, max_count):
        """Pick a small, spatially spread set of candidate waypoints."""
        if not candidates or max_count <= 0:
            return []

        start = self.current_pose or (0.0, 0.0)
        remaining = list(candidates)
        selected = [min(remaining, key=lambda item: self.distance_xy(start, item))]
        remaining.remove(selected[0])

        while remaining and len(selected) < max_count:
            next_item = max(
                remaining,
                key=lambda item: min(
                    self.distance_xy(item, chosen)
                    for chosen in selected
                ),
            )
            selected.append(next_item)
            remaining.remove(next_item)

        return selected

    def order_goals_nearest_neighbor(self, goals):
        """Order goals greedily by travel distance from the current pose."""
        remaining = list(goals)
        ordered = []
        current = self.current_pose or (0.0, 0.0)

        while remaining:
            next_goal = min(
                remaining,
                key=lambda goal: self.distance_xy(current, goal),
            )
            ordered.append(next_goal)
            remaining.remove(next_goal)
            current = (
                float(next_goal.get("x", 0.0)),
                float(next_goal.get("y", 0.0)),
            )

        for index, goal in enumerate(ordered[:-1]):
            next_goal = ordered[index + 1]
            dx = float(next_goal.get("x", 0.0)) - float(goal.get("x", 0.0))
            dy = float(next_goal.get("y", 0.0)) - float(goal.get("y", 0.0))
            goal["yaw"] = math.degrees(math.atan2(dy, dx))
        return ordered

    def distance_xy(self, a, b):
        """Return planar distance between a tuple/dict pair."""
        ax, ay = self.xy_from(a)
        bx, by = self.xy_from(b)
        return math.hypot(ax - bx, ay - by)

    def xy_from(self, value):
        """Extract x/y from either a dict goal or a tuple."""
        if isinstance(value, dict):
            return float(value.get("x", 0.0)), float(value.get("y", 0.0))
        return float(value[0]), float(value[1])

    def is_search_goal_list(self):
        """Return True when the active route is searching for unknown task points."""
        return bool(self.goals) and all(
            str(goal.get("task", "")).lower() == "search"
            for goal in self.goals
        )

    def send_next_goal(self):
        """Send the next Nav2 goal or finish the mission."""
        if self.mission_complete:
            return

        if self.time_remaining() <= 0:
            self.finish_mission("time_limit")
            return

        if self.is_search_goal_list() and not self.pending_tasks:
            self.finish_mission("all_tasks_found")
            return

        if self.goal_index >= len(self.goals):
            self.finish_mission("route_finished")
            return

        goal = self.goals[self.goal_index]

        if self.skip_navigation:
            self.get_logger().info(
                f"Skip navigation for goal {self.goal_index + 1}/{len(self.goals)}: "
                f"{goal.get('name')} task={goal.get('task')}"
            )
            self.start_inspection_delay()
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = "map"
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = float(goal.get("x", 0.0))
        nav_goal.pose.pose.position.y = float(goal.get("y", 0.0))

        yaw_rad = math.radians(float(goal.get("yaw", 0.0)))
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw_rad)
        nav_goal.pose.pose.orientation.x = qx
        nav_goal.pose.pose.orientation.y = qy
        nav_goal.pose.pose.orientation.z = qz
        nav_goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Goal {self.goal_index + 1}/{len(self.goals)}: {goal.get('name')} "
            f"task={goal.get('task')} x={goal.get('x')} y={goal.get('y')} "
            f"remaining={self.time_remaining():.1f}s"
        )
        future = self.nav_client.send_goal_async(nav_goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        """Handle Nav2 goal acceptance."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal was rejected.")
            self.record_navigation_failure("rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_navigation_result)

    def on_navigation_result(self, future):
        """Capture and inspect an image after Nav2 reaches a goal."""
        status = future.result().status
        goal = self.goals[self.goal_index]

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"Navigation failed with status {status}.")
            self.record_navigation_failure(f"status_{status}")
            return

        if self.wait_after_arrival > 0:
            self.inspect_timer = self.create_timer(
                self.wait_after_arrival,
                self.on_inspection_timer,
            )
            return

        self.complete_current_goal_inspection()

    def start_inspection_delay(self):
        """Delay capture without blocking camera callbacks."""
        if self.wait_after_arrival > 0:
            self.inspect_timer = self.create_timer(
                self.wait_after_arrival,
                self.on_inspection_timer,
            )
            return

        self.complete_current_goal_inspection()

    def on_inspection_timer(self):
        """Run one delayed inspection after camera frames have refreshed."""
        if self.inspect_timer is not None:
            self.inspect_timer.cancel()
            self.inspect_timer = None

        self.complete_current_goal_inspection()

    def complete_current_goal_inspection(self):
        """Inspect the current view, publish its result, and advance."""
        goal = self.goals[self.goal_index]
        if str(goal.get("task", "")).lower() == "search":
            results = self.inspect_search_view(goal)
            for result in results:
                self.record_result(result)
        else:
            result = self.inspect_current_view(goal)
            self.record_result(result)
        self.goal_index += 1
        self.send_next_goal()

    def record_navigation_failure(self, reason):
        """Publish a failure result and continue with the next goal."""
        goal = self.goals[self.goal_index]
        result = {
            "event": "inspection_result",
            "name": goal.get("name"),
            "task": goal.get("task"),
            "ok": False,
            "reason": reason,
        }
        self.record_result(result)
        self.goal_index += 1
        self.send_next_goal()

    def inspect_current_view(self, goal):
        """Run task-specific image analysis on the latest camera frame."""
        if self.latest_image is None:
            return {
                "event": "inspection_result",
                "name": goal.get("name"),
                "task": goal.get("task"),
                "ok": False,
                "reason": "no_image",
            }

        image = self.latest_image.copy()
        task = str(goal.get("task", "")).lower()
        raw_path = self.save_raw_image(goal, image)

        if task == "tower_light":
            payload, annotated = self.detect_tower_light(image)
        elif task == "baseball":
            payload, annotated = self.detect_orange_baseball(image)
        elif task == "multimeter":
            payload, annotated = self.detect_multimeters(image)
        elif task == "water_level":
            payload, annotated = self.detect_water_level(image)
        else:
            payload = {"ok": False, "reason": f"unknown_task:{task}"}
            annotated = image

        file_path = self.save_annotated_image(goal, annotated)
        self.publish_annotated_image(annotated)

        result = {
            "event": "inspection_result",
            "name": goal.get("name"),
            "task": task,
            "image_path": str(file_path),
            "raw_image_path": str(raw_path),
            "source": "local",
        }
        result.update(payload)
        return self.apply_vision_api_if_needed(task, result, str(raw_path))

    def apply_vision_api_if_needed(self, task, local_result, image_path):
        """Use Gemini API for required tasks or local-detector fallback."""
        local_complete = self.task_result_is_complete(task, local_result)
        api_required = task in self.vision_api_required_tasks
        api_fallback = task in self.vision_api_fallback_tasks

        if not api_required and not (api_fallback and not local_complete):
            return local_result

        api_result = self.call_vision_api(task, image_path, local_result)
        merged = dict(local_result)
        merged["local_result"] = local_result
        merged["api_result"] = api_result

        if api_result.get("ok"):
            merged.update(api_result)
            merged["source"] = "vision_api"
            return merged

        if api_required:
            merged["ok"] = False
            merged["source"] = "vision_api"
            merged["reason"] = api_result.get("reason", "vision_api_failed")
            return merged

        merged["source"] = "local"
        merged["vision_api_reason"] = api_result.get("reason", "vision_api_failed")
        return merged

    def call_vision_api(self, task, image_path, local_result):
        """Call the Gemini vision API service and return parsed JSON."""
        if not self.vision_api_client.wait_for_service(timeout_sec=0.2):
            return {
                "ok": False,
                "task": task,
                "source": "vision_api",
                "reason": "vision_api_service_unavailable",
            }

        request = VisionAnalyze.Request()
        request.request_json = json.dumps({
            "task": task,
            "image_path": image_path,
            "local_result": local_result,
        }, ensure_ascii=False)

        future = self.vision_api_client.call_async(request)
        deadline = time.time() + self.vision_api_timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            return {
                "ok": False,
                "task": task,
                "source": "vision_api",
                "reason": "vision_api_timeout",
            }

        try:
            return json.loads(future.result().response_json)
        except Exception as exc:
            return {
                "ok": False,
                "task": task,
                "source": "vision_api",
                "reason": f"vision_api_response_error:{exc}",
            }

    def inspect_search_view(self, goal):
        """Run only the current required detector from the current view."""
        if self.latest_image is None:
            return [{
                "event": "search_progress",
                "name": goal.get("name"),
                "task": "search",
                "ok": False,
                "reason": "no_image",
                "pending_tasks": self.ordered_pending_tasks(),
            }]

        results = []
        found_any = False
        task = self.current_task()
        if task is not None:
            task_goal = dict(goal)
            task_goal["task"] = task
            task_goal["name"] = f"{goal.get('name')}_{task}"
            result = self.inspect_current_view(task_goal)
            if self.task_result_is_complete(task, result):
                found_any = True
                result["search_goal"] = goal.get("name")
                result["time_remaining_sec"] = round(self.time_remaining(), 2)
                self.pending_tasks.discard(task)
                self.completed_tasks[task] = result
                results.append(result)

        if not found_any:
            annotated = self.latest_image.copy()
            self.draw_search_status(annotated, goal)
            file_path = self.save_annotated_image(goal, annotated, suffix="search")
            self.publish_annotated_image(annotated)
            results.append({
                "event": "search_progress",
                "name": goal.get("name"),
                "task": "search",
                "ok": True,
                "image_path": str(file_path),
                "current_task": task,
                "pending_tasks": self.ordered_pending_tasks(),
                "time_remaining_sec": round(self.time_remaining(), 2),
            })

        return results

    def draw_search_status(self, image, goal):
        """Overlay live mission status on a search frame."""
        lines = [
            f"search: {goal.get('name')}",
            f"current: {self.current_task()}",
            f"pending: {', '.join(self.ordered_pending_tasks())}",
            f"time: {self.time_remaining():.1f}s",
        ]
        y = 36
        for line in lines:
            cv2.putText(
                image,
                line,
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (255, 255, 255),
                2,
            )
            y += 32

    def record_result(self, result):
        """Publish and persist a mission result immediately."""
        task = str(result.get("task", "")).lower()
        if self.task_result_is_complete(task, result):
            self.pending_tasks.discard(task)
            self.completed_tasks[task] = result

        self.results.append(result)
        self.publish_result(result)
        live_path = self.save_dir / "live_results.json"
        live_path.write_text(
            json.dumps(self.results, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def finish_mission(self, reason):
        """Persist the final summary once."""
        if self.mission_complete:
            return

        return_to_start = bool(self.get_parameter("return_to_start").value)
        can_return = (
            return_to_start
            and not self.skip_navigation
            and not self.returning_to_start
            and self.start_pose is not None
            and reason != "time_limit"
        )
        if can_return:
            self.begin_return_to_start(reason)
            return

        self.mission_complete = True
        summary = {
            "event": "mission_complete",
            "reason": reason,
            "elapsed_sec": round(self.elapsed_time(), 2),
            "inspection_count": len(self.results),
            "task_order": list(MISSION_TASKS),
            "completed_tasks": [
                task for task in MISSION_TASKS
                if task in self.completed_tasks
            ],
            "pending_tasks": self.ordered_pending_tasks(),
            "returned_to_start": self.returning_to_start,
            "results": self.results,
        }
        summary_path = self.save_dir / "summary.json"
        summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        summary["summary_path"] = str(summary_path)
        self.publish_result(summary)
        self.get_logger().info(
            f"PBL inspection mission complete: {summary_path}"
        )

    def begin_return_to_start(self, reason):
        """Send the final Nav2 goal back to the recorded start pose."""
        self.returning_to_start = True
        self.return_reason = reason
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = "map"
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = float(self.start_pose[0])
        nav_goal.pose.pose.position.y = float(self.start_pose[1])
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 0.0)
        nav_goal.pose.pose.orientation.x = qx
        nav_goal.pose.pose.orientation.y = qy
        nav_goal.pose.pose.orientation.z = qz
        nav_goal.pose.pose.orientation.w = qw

        self.publish_result({
            "event": "return_to_start",
            "reason": reason,
            "x": float(self.start_pose[0]),
            "y": float(self.start_pose[1]),
            "time_remaining_sec": round(self.time_remaining(), 2),
        })
        future = self.nav_client.send_goal_async(nav_goal)
        future.add_done_callback(self.on_return_goal_response)

    def on_return_goal_response(self, future):
        """Handle final return-to-start goal acceptance."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Return-to-start goal was rejected.")
            self.finish_mission(f"{self.return_reason}_return_rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_return_result)

    def on_return_result(self, future):
        """Finish the mission after the robot returns to START."""
        status = future.result().status
        reason = self.return_reason or "complete"
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.finish_mission(f"{reason}_returned_to_start")
            return

        self.get_logger().warn(
            f"Return-to-start failed with status {status}."
        )
        self.finish_mission(f"{reason}_return_status_{status}")

    def current_task(self):
        """Return the next task required by the official A/B/C/D order."""
        for task in MISSION_TASKS:
            if task in self.pending_tasks:
                return task
        return None

    def ordered_pending_tasks(self):
        """Return pending tasks in official mission order."""
        return [task for task in MISSION_TASKS if task in self.pending_tasks]

    def elapsed_time(self):
        """Return elapsed mission seconds."""
        if self.mission_start_time is None:
            return 0.0
        return max(0.0, time.time() - self.mission_start_time)

    def time_remaining(self):
        """Return remaining mission seconds."""
        if self.mission_deadline is None:
            return float(self.get_parameter("mission_time_limit_sec").value)
        return max(0.0, self.mission_deadline - time.time())

    def task_result_is_complete(self, task, result):
        """Return True only when a result is good enough to stop searching."""
        if not result.get("ok"):
            return False

        if task == "tower_light":
            return result.get("light_color") in {"red", "yellow", "green"}
        if task == "baseball":
            return int(result.get("baseball_count", 0)) >= 1
        if task == "multimeter":
            return int(result.get("multimeter_count", 0)) >= 1
        if task == "water_level":
            return int(result.get("cup_count", 0)) >= 1 and all(
                level in {20, 40, 60, 80, 100}
                for level in result.get("water_level_percent", [])
            )
        return False

    def publish_result(self, result):
        """Publish one JSON result message."""
        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False)
        self.result_pub.publish(msg)
        self.get_logger().info(msg.data)

    def publish_annotated_image(self, image):
        """Publish the annotated inspection image."""
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.annotated_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"Annotated image publish failed: {exc}")

        if self.display_window:
            cv2.imshow("PBL Inspection", image)
            cv2.waitKey(1)

    def save_annotated_image(self, goal, image, suffix=None):
        """Save the annotated image for grading and debugging."""
        safe_name = str(goal.get("name", f"task_{self.goal_index + 1}"))
        safe_name = "".join(
            c if c.isalnum() or c in "_-" else "_"
            for c in safe_name
        )
        if suffix:
            safe_suffix = "".join(
                c if c.isalnum() or c in "_-" else "_"
                for c in str(suffix)
            )
            safe_name = f"{safe_name}_{safe_suffix}"
        timestamp = int(time.time() * 1000)
        path = self.save_dir / f"{self.goal_index + 1}_{safe_name}_{timestamp}.jpg"
        cv2.imwrite(str(path), image)
        return path

    def save_raw_image(self, goal, image):
        """Save an unannotated inspection image for API analysis and audit."""
        safe_name = str(goal.get("name", f"task_{self.goal_index + 1}"))
        safe_name = "".join(
            c if c.isalnum() or c in "_-" else "_"
            for c in safe_name
        )
        timestamp = int(time.time() * 1000)
        path = self.save_dir / f"{self.goal_index + 1}_{safe_name}_{timestamp}_raw.jpg"
        cv2.imwrite(str(path), image)
        return path

    def detect_tower_light(self, image):
        """Detect which signal tower light color is on."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        color_ranges = {
            "red": [
                ((0, 90, 80), (10, 255, 255)),
                ((170, 90, 80), (179, 255, 255)),
            ],
            "yellow": [((18, 80, 80), (38, 255, 255))],
            "green": [((40, 60, 60), (90, 255, 255))],
        }

        best = None
        best_area = 0.0
        best_box = None

        for color, ranges in color_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(
                    mask,
                    cv2.inRange(hsv, np.array(lower), np.array(upper)),
                )
            mask = self.clean_mask(mask, kernel_size=5)
            contour = self.largest_contour(mask)
            if contour is None:
                continue

            area = cv2.contourArea(contour)
            if area > best_area:
                best = color
                best_area = area
                best_box = cv2.boundingRect(contour)

        annotated = image.copy()
        if best_box is not None and best_area >= 50:
            self.draw_box(annotated, best_box, f"tower: {best}", (0, 255, 255))
            return {
                "ok": True,
                "light_color": best,
                "area": best_area,
            }, annotated

        cv2.putText(
            annotated,
            "tower: unknown",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2,
        )
        return {
            "ok": False,
            "light_color": "unknown",
            "area": best_area,
        }, annotated

    def detect_orange_baseball(self, image):
        """Find the orange baseball and draw its bounding box."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array((5, 80, 60)),
            np.array((25, 255, 255)),
        )
        mask = self.clean_mask(mask, kernel_size=5)
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 120:
                continue
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < 0.35:
                continue
            candidates.append((area * circularity, area, contour))

        annotated = image.copy()
        if not candidates:
            cv2.putText(
                annotated,
                "baseball: not found",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
            )
            return {
                "ok": False,
                "baseball_count": 0,
                "color": "orange",
            }, annotated

        candidates.sort(reverse=True, key=lambda item: item[0])
        _, area, contour = candidates[0]
        box = cv2.boundingRect(contour)
        self.draw_box(annotated, box, "orange baseball", (0, 140, 255))
        return {
            "ok": True,
            "baseball_count": 1,
            "color": "orange",
            "area": area,
            "box": self.box_to_list(box),
        }, annotated

    def detect_multimeters(self, image):
        """Count multimeters with YOLO first, then OpenCV as a fallback."""
        annotated = image.copy()
        boxes = self.detect_multimeters_with_yolo(image)
        method = "yolo"

        if boxes is None:
            boxes = self.detect_multimeters_with_opencv(image)
            method = "opencv_heuristic"

        for index, box in enumerate(boxes, start=1):
            self.draw_box(annotated, box, f"multimeter {index}", (255, 180, 0))

        cv2.putText(
            annotated,
            f"multimeters: {len(boxes)} ({method})",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
        )

        return {
            "ok": len(boxes) > 0,
            "multimeter_count": len(boxes),
            "method": method,
            "boxes": [self.box_to_list(box) for box in boxes],
        }, annotated

    def detect_multimeters_with_yolo(self, image):
        """Return multimeter boxes from the YOLO model if possible."""
        if self.yolo is None:
            return None

        try:
            results = self.yolo(image, conf=self.conf_thres, verbose=False)
        except Exception as exc:
            self.get_logger().warn(f"YOLO inference failed: {exc}")
            return None

        if not results or results[0].boxes is None:
            return []

        names = (
            getattr(results[0], "names", None)
            or getattr(self.yolo, "names", {})
        )
        boxes = []
        saw_supported_label = False

        for box in results[0].boxes:
            cls_id = int(box.cls[0].cpu().numpy())
            label = str(names.get(cls_id, cls_id)).lower()
            if label in self.multimeter_labels:
                saw_supported_label = True
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                boxes.append((int(x1), int(y1), int(x2 - x1), int(y2 - y1)))

        if not saw_supported_label and len(results[0].boxes) > 0:
            return None

        return boxes

    def detect_multimeters_with_opencv(self, image):
        """Detect likely multimeters by rectangular body plus circular dial."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        dark = cv2.inRange(gray, 0, 95)
        yellow = cv2.inRange(
            hsv,
            np.array((18, 55, 70)),
            np.array((45, 255, 255)),
        )
        mask = cv2.bitwise_or(dark, yellow)
        mask = self.clean_mask(mask, kernel_size=7)
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        boxes = []
        image_area = image.shape[0] * image.shape[1]
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < image_area * 0.002:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            aspect = w / float(h)
            if aspect < 0.35 or aspect > 1.35:
                continue

            crop = gray[y:y + h, x:x + w]
            if self.has_dial_like_circle(crop):
                boxes.append((x, y, w, h))

        return self.non_max_suppression(boxes)

    def has_dial_like_circle(self, gray_crop):
        """Check whether an object crop contains a central multimeter dial."""
        if gray_crop.size == 0:
            return False

        min_side = min(gray_crop.shape[:2])
        if min_side < 35:
            return False

        blurred = cv2.medianBlur(gray_crop, 5)
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=max(20, min_side // 3),
            param1=80,
            param2=18,
            minRadius=max(8, min_side // 12),
            maxRadius=max(12, min_side // 3),
        )
        return circles is not None

    def detect_water_level(self, image):
        """Count cups and estimate each cup's water level."""
        annotated = image.copy()
        h, w = image.shape[:2]

        if self.water_roi is None:
            scene_roi = (0, 0, w, h)
        else:
            x, y, rw, rh = self.water_roi
            x = max(0, min(x, w - 1))
            y = max(0, min(y, h - 1))
            rw = max(1, min(rw, w - x))
            rh = max(1, min(rh, h - y))
            scene_roi = (x, y, rw, rh)

        sx, sy, sw, sh = scene_roi
        cv2.rectangle(
            annotated,
            (sx, sy),
            (sx + sw, sy + sh),
            (255, 255, 255),
            2,
        )

        if self.cup_rois:
            cup_boxes = [
                self.clamp_box(box, w, h)
                for box in self.cup_rois
            ]
            cup_boxes = [box for box in cup_boxes if box is not None]
            detection_method = "calibrated_rois"
        else:
            cup_boxes = self.detect_cup_boxes(image, scene_roi)
            detection_method = "auto_water_regions"

        cups = []
        for index, box in enumerate(cup_boxes, start=1):
            cup = self.estimate_cup_water_level(image, box, index)
            cups.append(cup)

            color = (255, 0, 0) if cup["water_level_percent"] else (0, 0, 255)
            label = f"cup {index}: "
            if cup["water_level_percent"] is None:
                label += "unknown"
            else:
                label += f"{cup['water_level_percent']}%"
            self.draw_box(annotated, box, label, color)

            if cup.get("waterline_y") is not None:
                x, y, cw, _ = box
                waterline_y = int(cup["waterline_y"])
                cv2.line(
                    annotated,
                    (x, waterline_y),
                    (x + cw, waterline_y),
                    (255, 0, 0),
                    3,
                )

        if not cups:
            cv2.putText(
                annotated,
                "cups: 0",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
            )
            return {
                "ok": False,
                "cup_count": 0,
                "cups": [],
                "method": detection_method,
                "roi": self.box_to_list(scene_roi),
            }, annotated

        cv2.putText(
            annotated,
            f"cups: {len(cups)} ({detection_method})",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
        )

        water_levels = [cup["water_level_percent"] for cup in cups]
        ok = all(level is not None for level in water_levels)
        return {
            "ok": ok,
            "cup_count": len(cups),
            "water_level_percent": water_levels,
            "cups": cups,
            "method": detection_method,
            "roi": self.box_to_list(scene_roi),
        }, annotated

    def detect_cup_boxes(self, image, scene_roi):
        """Find likely cup boxes from colored water regions."""
        sx, sy, sw, sh = scene_roi
        roi = image[sy:sy + sh, sx:sx + sw]
        water_mask = self.water_color_mask(roi)
        contours, _ = cv2.findContours(
            water_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        boxes = []
        roi_area = sw * sh
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < max(80, roi_area * 0.0005):
                continue

            wx, wy, ww, wh = cv2.boundingRect(contour)
            if ww < 12 or wh < 8:
                continue

            cup_box = self.find_cup_box_around_water(roi, (wx, wy, ww, wh))
            cx, cy, cw, ch = cup_box
            boxes.append((sx + cx, sy + cy, cw, ch))

        boxes = self.non_max_suppression(boxes, overlap_threshold=0.45)
        boxes.sort(key=lambda item: item[0])
        return boxes

    def find_cup_box_around_water(self, roi, water_box):
        """Expand a water box to the surrounding cup outline when visible."""
        wx, wy, ww, wh = water_box
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 40, 120)
        edges = cv2.dilate(edges, np.ones((3, 3), dtype=np.uint8), iterations=1)
        contours, _ = cv2.findContours(
            edges,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        water_cx = wx + ww / 2.0
        water_cy = wy + wh / 2.0
        candidates = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if not (x <= water_cx <= x + w and y <= water_cy <= y + h):
                continue
            if w < ww * 0.8 or h < wh * 1.1:
                continue
            aspect = w / float(h)
            if aspect < 0.25 or aspect > 1.4:
                continue
            candidates.append((w * h, (x, y, w, h)))

        if candidates:
            _, box = max(candidates, key=lambda item: item[0])
            return self.pad_box(box, roi.shape[1], roi.shape[0], 0.08)

        estimated_h = int(max(wh * 2.2, ww * 1.45, 55))
        estimated_w = int(max(ww * 1.3, 28))
        x = int(wx + ww / 2 - estimated_w / 2)
        y = int(wy + wh - estimated_h)
        return self.clamp_box(
            (x, y, estimated_w, estimated_h),
            roi.shape[1],
            roi.shape[0],
        )

    def estimate_cup_water_level(self, image, cup_box, index):
        """Estimate one cup's water level and return a JSON-friendly payload."""
        x, y, w, h = cup_box
        crop = image[y:y + h, x:x + w]
        mask = self.water_color_mask(crop)
        ys, _ = np.where(mask > 0)

        method = "blue_cyan_mask"
        raw_percent = None
        waterline_y = None

        if len(ys) >= max(40, int(w * h * 0.01)):
            water_top = int(np.percentile(ys, 5))
            raw_percent = int(round((h - water_top) / float(h) * 100))
            waterline_y = y + water_top
        else:
            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            sobel_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
            row_score = np.mean(np.abs(sobel_y), axis=1)
            top_limit = int(h * 0.08)
            bottom_limit = int(h * 0.94)
            search_scores = row_score[top_limit:bottom_limit]
            method = "horizontal_edge"

            if search_scores.size > 0 and float(np.max(search_scores)) >= 4.0:
                water_top = int(np.argmax(search_scores) + top_limit)
                raw_percent = int(round((h - water_top) / float(h) * 100))
                waterline_y = y + water_top

        level = self.quantize_water_level(raw_percent)
        return {
            "index": int(index),
            "water_level_percent": level,
            "raw_percent": raw_percent,
            "box": self.box_to_list(cup_box),
            "method": method,
            "waterline_y": waterline_y,
        }

    def water_color_mask(self, image):
        """Return a cleaned mask for the blue/cyan water used in the task."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        blue = cv2.inRange(
            hsv,
            np.array((85, 45, 45)),
            np.array((130, 255, 255)),
        )
        cyan = cv2.inRange(
            hsv,
            np.array((70, 35, 55)),
            np.array((100, 255, 255)),
        )
        return self.clean_mask(cv2.bitwise_or(blue, cyan), kernel_size=5)

    def quantize_water_level(self, raw_percent):
        """Map a raw percentage to the allowed 20/40/60/80/100 labels."""
        if raw_percent is None:
            return None

        percent = int(round(max(0, min(raw_percent, 100)) / 20.0) * 20)
        return int(max(20, min(100, percent)))

    def clamp_box(self, box, width, height):
        """Clamp a box to image boundaries."""
        x, y, w, h = [int(v) for v in box]
        x = max(0, min(x, width - 1))
        y = max(0, min(y, height - 1))
        w = max(1, min(w, width - x))
        h = max(1, min(h, height - y))
        if w <= 0 or h <= 0:
            return None
        return (x, y, w, h)

    def pad_box(self, box, width, height, ratio):
        """Pad a box by a fraction of its size and clamp it to the image."""
        x, y, w, h = [int(v) for v in box]
        pad_x = int(w * ratio)
        pad_y = int(h * ratio)
        return self.clamp_box(
            (x - pad_x, y - pad_y, w + 2 * pad_x, h + 2 * pad_y),
            width,
            height,
        )

    def clean_mask(self, mask, kernel_size):
        """Remove small noise from a binary mask."""
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (kernel_size, kernel_size),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def largest_contour(self, mask):
        """Return the largest contour in a binary mask."""
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return None
        return max(contours, key=cv2.contourArea)

    def non_max_suppression(self, boxes, overlap_threshold=0.35):
        """Drop duplicate overlapping boxes."""
        if not boxes:
            return []

        boxes_np = np.array(boxes, dtype=float)
        x1 = boxes_np[:, 0]
        y1 = boxes_np[:, 1]
        x2 = boxes_np[:, 0] + boxes_np[:, 2]
        y2 = boxes_np[:, 1] + boxes_np[:, 3]
        area = boxes_np[:, 2] * boxes_np[:, 3]
        indexes = np.argsort(y2)
        picked = []

        while len(indexes) > 0:
            last = indexes[-1]
            picked.append(last)
            indexes = indexes[:-1]

            xx1 = np.maximum(x1[last], x1[indexes])
            yy1 = np.maximum(y1[last], y1[indexes])
            xx2 = np.minimum(x2[last], x2[indexes])
            yy2 = np.minimum(y2[last], y2[indexes])

            ww = np.maximum(0, xx2 - xx1)
            hh = np.maximum(0, yy2 - yy1)
            overlap = (ww * hh) / area[indexes]
            indexes = indexes[overlap <= overlap_threshold]

        return [boxes[i] for i in picked]

    def draw_box(self, image, box, label, color):
        """Draw one labeled bounding box."""
        x, y, w, h = [int(v) for v in box]
        cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
        cv2.putText(
            image,
            label,
            (x, max(24, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
        )

    def box_to_list(self, box):
        """Convert a bounding box tuple to a JSON-friendly list."""
        return [int(v) for v in box]


def main():
    """Run the autonomous PBL inspection node."""
    rclpy.init()
    node = PBLInspectionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
