# ASR-AMR

ASR-AMR is a ROS 2 workspace for an autonomous mobile robot built around
WHEELTEC robot components. It brings together robot base control, joystick and
keyboard teleoperation, lidar drivers, URDF models, SLAM, localization, and
Nav2-based autonomous navigation.

The workspace is organized as a standard `colcon` project:

```text
ASR-AMR/
|-- src/
|   |-- LLM_path_generator/     # AI path generate
|   |-- robot_controller/       # ASR base serial controller and launch files
|   |-- joy2cmd/                # Joystick-to-/cmd_vel bridge
|   |-- keyboard_control/       # Keyboard teleoperation helper
|   |-- object_tracking/        # HSV and YOLO visual target tracking
|   |-- path_planning/          # Fixed Nav2 goal sender example
|   |-- tracking_msg/           # Custom visual tracking message interface
|   |-- wheeltec_robot_nav2/    # Nav2 bringup, maps, params, RViz config
|   |-- wheeltec_robot_slam/    # gmapping, Cartographer, slam_toolbox wrappers
|   |-- wheeltec_robot_urdf/    # Robot URDFs, meshes, RViz model config
|   `-- wheeltec_lidar_ros2/    # LDlidar and Leishen lidar drivers
|-- .gitignore
`-- README.md
```

## Features

- Serial base controller for the ASR robot body.
- Publishes odometry on `/odom` and raw IMU data on `/imu/data_raw`.
- Subscribes to `/cmd_vel` for velocity commands.
- LSN10P lidar launch and configuration through `lslidar_driver`.
- Nav2 localization and navigation using the bundled `WHEELTEC` map.
- EKF fusion through `robot_localization`.
- Joystick control through `joy` and `joy2cmd`.
- Keyboard teleoperation through `keyboard_control`.
- Visual target tracking with HSV thresholding or YOLO.
- Visual servo control from target yaw/depth errors to `/cmd_vel`.
- Custom `tracking_msg/msg/Imgtracking` interface for target validity,
  normalized yaw error, depth, confidence, and class ID.
- Optional SLAM packages for gmapping, Cartographer, and slam_toolbox.

## Main Packages

| Package | Purpose |
| --- | --- |
| `robot_controller` | Launches the robot state publisher, static transforms, EKF, IMU filter, lidar, and ASR serial controller. |
| `wheeltec_nav2` | Nav2 launch files, map files, behavior tree XML, and robot-specific navigation parameters. |
| `wheeltec_robot_urdf` | WHEELTEC and ASR URDF models and meshes. |
| `lslidar_driver` / `lslidar_msgs` | Leishen lidar driver and message definitions. |
| `ldlidar_stl_ros2` / `ldlidar_sl_ros2` | LD06, LD14, and LD19 lidar drivers. |
| `joy2cmd` | Converts `/joy` joystick messages to `/cmd_vel`. |
| `keyboard_control` | Publishes simple keyboard-driven `/cmd_vel` messages. |
| `object_tracking` | Camera-based target detection, HSV tuning, YOLO detection, and visual servo task control. |
| `tracking_msg` | Defines `Imgtracking`, the message shared by the detector and tracking controller. |
| `path_planning` | Sends a fixed `NavigateToPose` goal to Nav2. |
| `slam_gmapping`, `wheeltec_cartographer`, `wheeltec_slam_toolbox` | SLAM-related packages and launch wrappers. |

## Requirements

This project expects a Linux machine with ROS 2 installed. The code uses ROS 2
Python launch files, `ament_cmake`, `ament_python`, Nav2, and common robot
drivers. It has been structured for a physical robot, so some launch files
expect hardware serial devices to exist.

Common dependencies include:

```bash
# Replace humble with your installed ROS 2 distro if needed.
export ROS_DISTRO=humble

sudo apt update
sudo apt install \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-serial \
  ros-${ROS_DISTRO}-navigation2 \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-robot-localization \
  ros-${ROS_DISTRO}-imu-filter-madgwick \
  ros-${ROS_DISTRO}-joint-state-publisher \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-joy \
  ros-${ROS_DISTRO}-tf-transformations \
  ros-${ROS_DISTRO}-slam-toolbox \
  python3-opencv \
  libpcap-dev \
  libpcl-dev
```

The YOLO visual tracker also requires the Python `ultralytics` package:

```bash
python3 -m pip install ultralytics
```

You can also ask `rosdep` to install dependencies declared by the packages:

```bash
cd /path/to/ASR-AMR
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Some package manifests are still minimal, so install the common dependencies
above if `rosdep` does not catch everything.

## Build

From the workspace root:

```bash
cd /path/to/ASR-AMR
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If you only want to rebuild the project-specific Python packages while working:

```bash
colcon build --symlink-install --packages-select \
  robot_controller joy2cmd keyboard_control path_planning tracking_msg object_tracking
source install/setup.bash
```

## Hardware Notes

Check these settings before launching on a robot:

- The ASR base controller opens `/dev/ttyTHS1` at `115200` baud in
  `src/robot_controller/robot_controller/ASR_robot.py`.
- The default LSN10P lidar configuration uses `/dev/ttyACM0` in
  `src/wheeltec_lidar_ros2/LSlidar/lslidar_driver/params/lidar_uart_ros2/lsn10p.yaml`.
- Joystick control expects the ROS 2 `joy_node` package and a connected gamepad.
- The main robot model launched by `robot_controller` is `ASR_robot.urdf`.

If your serial devices use different names, update the relevant file or create
stable udev aliases before launching.

## Launch The Robot Base

This starts the ASR robot controller, URDF publishing, static transforms, EKF,
IMU filter, joystick nodes, and the configured LSN10P lidar:

```bash
source install/setup.bash
ros2 launch robot_controller ASR_robot.launch.py
```

For the base stack without joystick nodes:

```bash
ros2 launch robot_controller ASR_robot_nav.launch.py
```

For joystick-only control with the ASR controller:

```bash
ros2 launch robot_controller AMR_Joy_Control.launch.py
```

## Teleoperation

Joystick control is available through `joy2cmd`. The current mapping is:

- `axes[4]`: forward/backward velocity, scaled to `linear.x`.
- `axes[3]`: left/right velocity, scaled to `linear.y`.
- `axes[0]`: yaw velocity, scaled to `angular.z`.

Run it directly after starting `joy_node`:

```bash
ros2 run joy joy_node
ros2 run joy2cmd joy2cmd
```

Keyboard control publishes simple velocity commands:

```bash
ros2 run keyboard_control robot_keyboard_control
```

Keyboard keys:

| Key | Command |
| --- | --- |
| `w` | Move forward |
| `s` | Move backward |
| `a` | Strafe left |
| `d` | Strafe right |
| `Ctrl-C` | Exit |

## Visual Object Tracking

The `object_tracking` package provides two detector nodes and one controller
node. Both detectors publish `tracking_msg/msg/Imgtracking` on
`/Visual_sensor_vel`, and `robot_task` subscribes to that topic to publish
visual-servo velocity commands on `/cmd_vel`.

Camera topics expected by the detector nodes:

| Topic | Type | Description |
| --- | --- | --- |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB image stream. |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | Depth image aligned to the RGB frame. |

YOLO tracking uses the bundled model at
`src/object_tracking/object_tracking/best.pt` by default. It keeps the
highest-confidence detection above the configured threshold, estimates distance
from the center area of the aligned depth image, and publishes:

- `target_visible`: true only when a usable target and depth are available.
- `yaw`: normalized horizontal error, where positive means the target is left of
  image center.
- `depth`: target distance in meters, or `-1.0` when invalid.
- `confidence`: selected detector confidence.
- `class_id`: selected detector class ID.

Run YOLO detection:

```bash
source install/setup.bash
ros2 run object_tracking object_tracking_YOLO
```

You can override the model path or confidence threshold:

```bash
ros2 run object_tracking object_tracking_YOLO --ros-args \
  -p model_path:=/path/to/best.pt \
  -p confidence_threshold:=0.50
```

Run HSV color tracking instead:

```bash
ros2 run object_tracking object_tracking_img
```

Tune HSV thresholds and save them to
`src/object_tracking/object_tracking/hsv_values.json`:

```bash
ros2 run object_tracking hsv_adjust
```

Run the visual servo controller after starting one detector:

```bash
ros2 run object_tracking robot_task
```

`robot_task` stops the robot when target data is stale, no target is visible, or
depth is invalid. It also stops when the target is closer than `0.35 m`.

## Navigation

The main Nav2 launch file starts the robot base stack and then launches
localization and navigation:

```bash
source install/setup.bash
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py \
  map:=$(ros2 pkg prefix wheeltec_nav2)/share/wheeltec_nav2/map/WHEELTEC.yaml \
  params_file:=$(ros2 pkg prefix wheeltec_nav2)/share/wheeltec_nav2/param/wheeltec_param/ASR_robot.yaml
```

The explicit `map:=...` argument is recommended because one default path in
`wheeltec_nav2.launch.py` points to `/home/robot/Robot/...`, which may not exist
on your machine.

To send the example fixed navigation goal:

```bash
ros2 run path_planning path_generator
```

The current goal is set in
`src/path_planning/path_planning/path_generator.py`.

## SLAM

Available SLAM launch files include:

```bash
ros2 launch slam_gmapping slam_gmapping.launch.py
ros2 launch wheeltec_cartographer cartographer.launch.py
ros2 launch wheeltec_slam_toolbox online_sync.launch.py
ros2 launch wheeltec_slam_toolbox online_async_launch.py
```

Some SLAM launch files were adapted from WHEELTEC examples and may need package
name or topic adjustments for this workspace before use on a new machine.

To save a Nav2 map:

```bash
ros2 launch wheeltec_nav2 save_map.launch.py
```

## Important Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command input for the base controller. |
| `/odom` | `nav_msgs/msg/Odometry` | Odometry published by `ASR_robot`. |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | Raw IMU data from the ASR controller. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Lidar scan topic used by Nav2 and SLAM. |
| `/joy` | `sensor_msgs/msg/Joy` | Joystick input used by `joy2cmd`. |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB input for visual tracking. |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | Aligned depth input for visual tracking. |
| `/Visual_sensor_vel` | `tracking_msg/msg/Imgtracking` | Visual target state from `object_tracking` detector nodes. |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 action server used by `path_planning`. |

## Troubleshooting

### Cannot open serial port

Check the actual device name:

```bash
ls /dev/ttyTHS* /dev/ttyACM* /dev/ttyUSB*
```

Then update the ASR controller or lidar YAML file to match your hardware.
You may also need to add your user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
```

Log out and back in after changing group membership.

### Nav2 cannot find the map

Pass the map path explicitly:

```bash
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py \
  map:=$(ros2 pkg prefix wheeltec_nav2)/share/wheeltec_nav2/map/WHEELTEC.yaml
```

### Lidar data is missing

Verify the lidar port in:

```text
src/wheeltec_lidar_ros2/LSlidar/lslidar_driver/params/lidar_uart_ros2/lsn10p.yaml
```

Then check that `/scan` is publishing:

```bash
ros2 topic echo /scan
```

### Visual tracking is not publishing

Check that the camera topics exist:

```bash
ros2 topic list | grep camera
```

Then inspect the tracking output:

```bash
ros2 topic echo /Visual_sensor_vel
```

If the YOLO node fails at startup, confirm that `ultralytics` is installed and
that `best.pt` exists in the `object_tracking` package.

## Development Notes

- Generated workspace folders (`build/`, `install/`, and `log/`) are ignored by
  git.
- Build `tracking_msg` before running `object_tracking` nodes that import
  `tracking_msg.msg.Imgtracking`.
- Use `colcon build --symlink-install` while iterating on Python packages.
- Source `install/setup.bash` in every new terminal before running package
  commands.
- Keep robot-specific serial ports, maps, and Nav2 parameters synchronized with
  the physical platform before testing autonomous navigation.
