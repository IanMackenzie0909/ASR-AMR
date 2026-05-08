import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('robot_controller')

    robot_controller_node = launch_ros.actions.Node(
        package='robot_controller',        # 你的 package 名稱
        executable='ASR_robot',        # 直接指定 Python 檔名
        name='ASR_robot',                 # ROS node name
        output='screen',                  # 顯示在終端
        emulate_tty=True                  # 讓 print 可以即時顯示
    )

    joy_node = launch_ros.actions.Node(
        package='joy',        # 你的 package 名稱
        executable='joy_node',        # 直接指定 Python 檔名
        name='joy_node',                 # ROS node name
        output='screen',                  # 顯示在終端
        emulate_tty=True                  # 讓 print 可以即時顯示
    )

    joy2cmd_node = launch_ros.actions.Node(
        package='joy2cmd',        # 你的 package 名稱
        executable='joy2cmd',        # 直接指定 Python 檔名
        name='joy2cmd',                 # ROS node name
        output='screen',                  # 顯示在終端
        emulate_tty=True                  # 讓 print 可以即時顯示
    )

    ld = LaunchDescription()

    ld.add_action(robot_controller_node)
    ld.add_action(joy_node)
    ld.add_action(joy2cmd_node)

    return ld

