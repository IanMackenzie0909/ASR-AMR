"""Launch the autonomous PBL inspection mission node."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    """Create the PBL inspection node after reading the goals JSON file."""
    goals_file = Path(LaunchConfiguration("goals_file").perform(context))
    goals_json = goals_file.read_text(encoding="utf-8")
    nodes = []

    if LaunchConfiguration("start_vision_api").perform(context).lower() == "true":
        nodes.append(
            Node(
                package="object_tracking",
                executable="pbl_vision_api",
                name="pbl_vision_api",
                output="screen",
                parameters=[
                    {
                        "gemini_model": LaunchConfiguration("gemini_model"),
                        "api_timeout_sec": ParameterValue(
                            LaunchConfiguration("vision_api_timeout_sec"),
                            value_type=float,
                        ),
                        "max_api_calls": ParameterValue(
                            LaunchConfiguration("vision_api_max_calls"),
                            value_type=int,
                        ),
                        "save_dir": LaunchConfiguration("save_dir"),
                        "mock_mode": ParameterValue(
                            LaunchConfiguration("vision_api_mock_mode"),
                            value_type=bool,
                        ),
                    }
                ],
            )
        )

    nodes.append(
        Node(
            package="object_tracking",
            executable="pbl_inspection",
            name="pbl_inspection",
            output="screen",
            parameters=[
                {
                    "goals_json": goals_json,
                    "mission_mode": LaunchConfiguration("mission_mode"),
                    "mission_time_limit_sec": ParameterValue(
                        LaunchConfiguration("mission_time_limit_sec"),
                        value_type=float,
                    ),
                    "optimize_goal_order": ParameterValue(
                        LaunchConfiguration("optimize_goal_order"),
                        value_type=bool,
                    ),
                    "return_to_start": ParameterValue(
                        LaunchConfiguration("return_to_start"),
                        value_type=bool,
                    ),
                    "map_yaml_file": LaunchConfiguration("map_yaml_file"),
                    "search_waypoints_json": LaunchConfiguration(
                        "search_waypoints_json"
                    ),
                    "search_yaw_offsets_json": LaunchConfiguration(
                        "search_yaw_offsets_json"
                    ),
                    "search_spacing_m": ParameterValue(
                        LaunchConfiguration("search_spacing_m"),
                        value_type=float,
                    ),
                    "search_max_waypoints": ParameterValue(
                        LaunchConfiguration("search_max_waypoints"),
                        value_type=int,
                    ),
                    "search_clearance_m": ParameterValue(
                        LaunchConfiguration("search_clearance_m"),
                        value_type=float,
                    ),
                    "model_path": LaunchConfiguration("model_path"),
                    "confidence_threshold": ParameterValue(
                        LaunchConfiguration("confidence_threshold"),
                        value_type=float,
                    ),
                    "save_dir": LaunchConfiguration("save_dir"),
                    "wait_after_arrival_sec": ParameterValue(
                        LaunchConfiguration("wait_after_arrival_sec"),
                        value_type=float,
                    ),
                    "skip_navigation": ParameterValue(
                        LaunchConfiguration("skip_navigation"),
                        value_type=bool,
                    ),
                    "water_roi_json": LaunchConfiguration("water_roi_json"),
                    "cup_rois_json": LaunchConfiguration("cup_rois_json"),
                    "display_window": ParameterValue(
                        LaunchConfiguration("display_window"),
                        value_type=bool,
                    ),
                    "vision_api_service": LaunchConfiguration("vision_api_service"),
                    "vision_api_timeout_sec": ParameterValue(
                        LaunchConfiguration("vision_api_timeout_sec"),
                        value_type=float,
                    ),
                    "vision_api_required_tasks_json": LaunchConfiguration(
                        "vision_api_required_tasks_json"
                    ),
                    "vision_api_fallback_tasks_json": LaunchConfiguration(
                        "vision_api_fallback_tasks_json"
                    ),
                    "camera_topic": LaunchConfiguration("camera_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "amcl_pose_topic": LaunchConfiguration("amcl_pose_topic"),
                }
            ],
        )
    )
    return nodes


def generate_launch_description():
    """Return launch arguments and the configured node."""
    package_share = get_package_share_directory("object_tracking")
    try:
        nav2_share = get_package_share_directory("wheeltec_nav2")
        default_map_yaml = os.path.join(nav2_share, "map", "WHEELTEC.yaml")
    except Exception:
        default_map_yaml = "src/wheeltec_robot_nav2/map/WHEELTEC.yaml"

    default_goals_file = os.path.join(
        package_share,
        "config",
        "pbl_goals.example.json",
    )
    default_model_path = os.path.join(
        package_share,
        "object_tracking",
        "best.pt",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "goals_file",
                default_value=default_goals_file,
                description="JSON file containing four Nav2 inspection goals.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=default_model_path,
                description="YOLO model used for multimeter detection.",
            ),
            DeclareLaunchArgument(
                "mission_mode",
                default_value="auto",
                description="auto, fixed, or search.",
            ),
            DeclareLaunchArgument("mission_time_limit_sec", default_value="175.0"),
            DeclareLaunchArgument("optimize_goal_order", default_value="false"),
            DeclareLaunchArgument("return_to_start", default_value="true"),
            DeclareLaunchArgument("map_yaml_file", default_value=default_map_yaml),
            DeclareLaunchArgument("search_waypoints_json", default_value=""),
            DeclareLaunchArgument(
                "search_yaw_offsets_json",
                default_value="[0,90,180,-90]",
            ),
            DeclareLaunchArgument("search_spacing_m", default_value="2.0"),
            DeclareLaunchArgument("search_max_waypoints", default_value="4"),
            DeclareLaunchArgument("search_clearance_m", default_value="0.35"),
            DeclareLaunchArgument("confidence_threshold", default_value="0.50"),
            DeclareLaunchArgument("save_dir", default_value="/tmp/pbl_inspection"),
            DeclareLaunchArgument("wait_after_arrival_sec", default_value="0.4"),
            DeclareLaunchArgument("skip_navigation", default_value="false"),
            DeclareLaunchArgument("water_roi_json", default_value=""),
            DeclareLaunchArgument("cup_rois_json", default_value=""),
            DeclareLaunchArgument("display_window", default_value="false"),
            DeclareLaunchArgument("start_vision_api", default_value="true"),
            DeclareLaunchArgument("vision_api_service", default_value="/pbl_vision_api/analyze"),
            DeclareLaunchArgument("vision_api_timeout_sec", default_value="8.0"),
            DeclareLaunchArgument("vision_api_max_calls", default_value="4"),
            DeclareLaunchArgument("vision_api_mock_mode", default_value="false"),
            DeclareLaunchArgument(
                "vision_api_required_tasks_json",
                default_value='["water_level"]',
            ),
            DeclareLaunchArgument(
                "vision_api_fallback_tasks_json",
                default_value='["multimeter"]',
            ),
            DeclareLaunchArgument(
                "gemini_model",
                default_value="gemini-3-flash-preview",
            ),
            DeclareLaunchArgument(
                "camera_topic",
                default_value="/camera/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument("amcl_pose_topic", default_value="/amcl_pose"),
            OpaqueFunction(function=launch_setup),
        ]
    )
