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

def generate_robot_node(robot_urdf):
    return launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'), 'urdf', robot_urdf)],
    )

def generate_static_transform_publisher_node(translation, rotation, parent, child):
    return launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'base_to_{child}',
        arguments=[translation[0], translation[1], translation[2], rotation[0], rotation[1], rotation[2], parent, child],
    )

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('robot_controller')
    launch_dir = os.path.join(bringup_dir, 'launch')
        
    ekf_config = Path(get_package_share_directory('robot_controller'), 'config', 'ekf.yaml')
    ekf_carto_config = Path(get_package_share_directory('robot_controller'), 'config', 'ekf_carto.yaml')

    imu_config = Path(get_package_share_directory('robot_controller'), 'config', 'imu.yaml')

    
    carto_slam = LaunchConfiguration('carto_slam', default='false')
    
    carto_slam_dec = DeclareLaunchArgument('carto_slam',default_value='false')
            
#     wheeltec_robot = IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
#             launch_arguments={'akmcar': 'false'}.items(),
#     )
#     #choose your car,the default car is mini_mec 
#     choose_car = IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description.launch.py')),
#     )
        
    robot_ekf = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_ekf.launch.py')),
            launch_arguments={'carto_slam':carto_slam}.items(),            
    )

                                                            
    base_to_link = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_link',
            arguments=['0', '0', '0','0', '0','0','base_footprint','base_link'],
    )
    base_to_gyro = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_gyro',
            arguments=['0', '0', '0','0', '0','0','base_footprint','gyro_link'],
    )

    mini_mec = GroupAction([
            generate_robot_node('ASR_robot.urdf'),
            generate_static_transform_publisher_node(['0.165', '0', '0.12'], ['0', '0', '0'], 'base_footprint', 'laser'),
            generate_static_transform_publisher_node(['0.165', '0', '0.12'], ['0', '0', '0'], 'base_footprint', 'camera_link'),
    ])
    
    imu_filter_node =  launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_config]
    )
    
                           
    joint_state_publisher_node = launch_ros.actions.Node(
            package='joint_state_publisher', 
            executable='joint_state_publisher', 
            name='joint_state_publisher',
    )

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

    lslidar_dir = get_package_share_directory('lslidar_driver')
    # 建立 IncludeLaunchDescription
    lsn10p_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lslidar_dir, 'launch', 'lsn10p_launch.py')
        )
    )

    # # SLAM gmapping
    # slam_dir = get_package_share_directory('slam_gmapping')
    # gmapping_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(slam_dir, 'launch', 'slam_gmapping.launch.py')
    #     )
    # )


    ld = LaunchDescription()

#     ld.add_action(choose_car)
    ld.add_action(carto_slam_dec)
#     ld.add_action(wheeltec_robot)
    ld.add_action(base_to_link)
    ld.add_action(base_to_gyro)
    ld.add_action(mini_mec)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(imu_filter_node)    
    ld.add_action(robot_ekf)
    ld.add_action(robot_controller_node)
    ld.add_action(joy_node)
    ld.add_action(joy2cmd_node)
    ld.add_action(lsn10p_launch)
    # ld.add_action(gmapping_launch)

    return ld

