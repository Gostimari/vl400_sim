# vl400_sim (ROS 2)

Gazebo (gz-sim / Harmonic) simulation of the **VL400** tracked mulcher
robot in a forest world. The robot is a ~2.5 t skid-steer tracked
vehicle with animated tread plates, a front mulcher, a Livox HAP lidar,
a ZED2i stereo camera, an IMU/GPS/magnetometer and a VLP-16 lidar — all
bridged to **ROS 2 Lyrical** via `ros_gz`.

## Layout

```
urdf/vl400.xacro              main robot: tracks, mulcher, controllers, tread animation hooks
urdf/sentry.xacro             top "sentry" box: IMU, GPS, magnetometer, VLP-16 mount
urdf/zed2i_camera.xacro       ZED2i stereo camera + IMU (native gz-sim sensors)
urdf/livox_hap.xacro         Livox HAP lidar (gpu_lidar + custom Livox-pattern plugin)
urdf/velodyne_vlp16.urdf.xacro  VLP-16 lidar macro
worlds/                       gz-sim worlds (see "Choosing a world")
models/                       forest props (tree, grass, ground, rock, bush, ...)
launch/gazebo.launch.py       sim + ros_gz bridge + robot_state_publisher + helper
config/bridge.yaml            ros_gz_bridge topic mappings
scan_patterns/                Livox HAP scan pattern (HAP.csv)
src/ include/                 custom gz-sim systems: track tread animation,
                              Livox lidar simulation
scripts/                      magnetometer_for_simulation.py helper
meshes/                       robot / prop mesh assets
```

## Cloning this repository (git LFS)

All heavy mesh assets (`*.stl`, `*.STL`, `*.dae`, `*.obj`, `*.glb`,
`*.zip` — the robot meshes and the ~3.7 GB terrain models) are stored
with **git LFS**. Install and enable it **before** the first clone or
pull, otherwise the working tree gets 130-byte LFS pointer files
instead of the real meshes:

```bash
sudo apt install git-lfs
git lfs install          # once per user / per clone
git clone git@github.com:Gostimari/vl400_sim.git   # then clone or pull
```

How to tell the pointers are there (the simulation does not launch):

- Mesh files are tiny (~130 B) and start with
  `version https://git-lfs.github.com/spec/v1`.
- gz-sim logs `Unable to read STL` / `Unable to load collada file`
  and then crashes with `ODE INTERNAL ERROR 1 ... "vertices" failed`.

Recovery if it ever happens:

```bash
git lfs pull            # fetch from remote + rewrite the working tree
git lfs checkout        # or: restore from the local cache (.git/lfs)
colcon build --packages-select vl400_sim   # install/ copies the meshes
```

Rebuild after any pull/checkout that touches the mesh files — `install/`
holds copies, and gz-sim loads the meshes from there, not from `src/`.

## Dependencies

All dependencies are declared in `package.xml`. The recommended way to
install them is `rosdep` (it reads `package.xml` directly, so this list
can't drift out of sync):

```bash
sudo apt install python3-rosdep
rosdep init      # only the first time
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Or install them explicitly (ROS 2 Lyrical):

```bash
sudo apt install \
  ros-lyrical-ament-cmake \
  ros-lyrical-rosidl-default-generators ros-lyrical-rosidl-default-runtime \
  ros-lyrical-std-msgs ros-lyrical-sensor-msgs ros-lyrical-geometry-msgs \
  ros-lyrical-nav-msgs ros-lyrical-rosgraph-msgs \
  ros-lyrical-rclcpp ros-lyrical-rclpy ros-lyrical-tf2-ros \
  ros-lyrical-ament-index-cpp \
  ros-lyrical-gz-sim-vendor ros-lyrical-gz-math-vendor \
  ros-lyrical-gz-msgs-vendor ros-lyrical-gz-transport-vendor \
  ros-lyrical-gz-plugin-vendor \
  ros-lyrical-ros-gz-sim ros-lyrical-ros-gz-bridge \
  ros-lyrical-xacro ros-lyrical-robot-state-publisher \
  ros-lyrical-joint-state-publisher ros-lyrical-joint-state-publisher-gui \
  ros-lyrical-rviz2 \
  python3-scipy
```

Notes:
- `ros-lyrical-ros-gz` is a metapackage that pulls in `ros-gz-sim`,
  `ros-gz-bridge` and more — installing the two packages above is enough.
- The `ros-lyrical-gz-*-vendor` packages provide the gz-sim C++ libraries
  (`gz-sim`, `gz-math`, `gz-msgs`, `gz-transport`, `gz-plugin`) that this
  package's custom plugins link against.
- `ros-lyrical-joint-state-publisher(_gui)` and `python3-scipy` are
  declared in `package.xml` but not required to run this simulation
  (joint states come from gz-sim, and no helper node uses scipy).

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select vl400_sim
source install/setup.bash
```

## Run

```bash
# Full simulation (default world: forest_ground-100-easy)
ros2 launch vl400_sim gazebo.launch.py

# Headless (server only, offscreen sensor rendering)
ros2 launch vl400_sim gazebo.launch.py headless:=true
```

Launch arguments:

| arg | default | meaning |
|---|---|---|
| `world` | `worlds/forest_ground-100-easy.world` | world file to load |
| `headless` | `false` | no GUI, offscreen rendering |
| `gpu` | `true` | GPU-accelerated sensors (lidar/camera) |
| `organize_cloud` | `false` | organize the VLP-16 cloud into rows (2D PointCloud2) |
| `x` `y` `z` | `4` `10` `5` | robot spawn pose; `z` is the drop height |

`z` is how high the robot is dropped at spawn — if it tumbles on spawn,
lower it (e.g. `z:=3.4`).

## Choosing a world

Worlds live in `worlds/`. The `model://` props (trees, ground, rocks, ...)
are resolved automatically from this package's `models/` directory, so you
only need to point the `world` argument at a `.world` file.

| file | world name (internal) | description |
|---|---|---|
| `forest_ground-50-easy.world` | `forest_ground-50-easy` | forest, 50 size, easy |
| `forest_ground-50-medium.world` | `forest_ground-50-medium` | forest, 50 size, medium |
| `forest_ground-50-hard.world` | `forest_ground-50-hard` | forest, 50 size, hard |
| `forest_ground-100-easy.world` | `forest_ground-100-easy` | forest, 100 size, easy (**default**) |
| `forest_ground-100-medium.world` | `forest_ground-100-medium` | forest, 100 size, medium |
| `forest_ground-100-hard.world` | `forest_ground-100-hard` | forest, 100 size, hard |
| `forest_ground-200-easy.world` | `forest_ground-200-easy` | forest, 200 size, easy |
| `forest_ground-200-medium.world` | `forest_ground-200-medium` | forest, 200 size, medium |
| `forest_ground-200-hard.world` | `forest_ground-200-hard` | forest, 200 size, hard |
| `robot.world` | `default` | flat test world |

```bash
# Example: run the 200-tree hard forest
ros2 launch vl400_sim gazebo.launch.py \
  world:=/home/duarte/ros2_ws/src/vl400_sim/worlds/forest_ground-200-hard.world
```

> **Joint states and the world.** The `/joint_states` bridge entry in
> `config/bridge.yaml` is world-scoped: gz publishes it as
> `/world/<world_name>/model/vl400/joint_state`, where `<world_name>` is
> the **internal** `<world name="...">` attribute of the world file
> (see the table above — note `robot.world` is internally named
> `default`). The launch patches this name automatically from the
> `world` argument, so switching worlds needs no manual edits — if
> `/joint_states` ever looks empty, check that the world you passed
> resolves and that you rebuilt after your last `src/` change:
> ```bash
> colcon build --packages-select vl400_sim
> ```
> (The launch reads worlds/urdf/config from `install/`, so after editing
> any of those files in `src/` you must rebuild.)

## ROS topics

Bridged from Gazebo (`config/bridge.yaml`):

| ROS topic | Type | Source |
|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | ROS -> tracked-vehicle controller |
| `/odom` | nav_msgs/Odometry | slip-free kinematic odometry |
| `/odom_tracked` | nav_msgs/Odometry | ground-truth odometry (incl. track slip) |
| `/joint_states` | sensor_msgs/JointState | all robot joints |
| `/imu` | sensor_msgs/Imu | IMU on the sentry box |
| `/fix` | sensor_msgs/NavSatFix | GPS |
| `/mag` | sensor_msgs/MagneticField | magnetometer |
| `/clock` | rosgraph_msgs/Clock | sim time (use_sim_time) |
| `/velodyne_points` | sensor_msgs/PointCloud2 | VLP-16 cloud |
| `/livox_hap_points` | sensor_msgs/PointCloud2 | raw gz ray cloud of the Livox HAP |
| `/livox/lidar/pointcloud2` | sensor_msgs/PointCloud2 | Livox-pattern cloud (published by the lidar plugin directly, no bridge) |

ZED2i stereo camera (`urdf/zed2i_camera.xacro`):

| ROS topic | Type |
|---|---|
| `/zed2/left_raw/image_raw_color` | sensor_msgs/Image |
| `/zed2/left_raw/camera_info` | sensor_msgs/CameraInfo |
| `/zed2/right_raw/image_raw_color` | sensor_msgs/Image |
| `/zed2/right_raw/camera_info` | sensor_msgs/CameraInfo |
| `/zed2/right/image_rect_color` | sensor_msgs/Image |
| `/zed2/right/camera_info` | sensor_msgs/CameraInfo |
| `/zed2/depth/depth_registered` | sensor_msgs/Image (32FC1) |
| `/zed2/depth/camera_info` | sensor_msgs/CameraInfo |
| `/zed2/point_cloud/cloud_registered` | sensor_msgs/PointCloud2 |
| `/zed2/imu` | sensor_msgs/Imu |

Drive the robot (skid-steer):

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"   # forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"  # turn
```
or `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.

Helper node: `magnetometer_for_simulation.py` publishes the compass
heading (deg) on `heading`.

## Sensor frames (TF, from robot_state_publisher)

- Livox HAP: `livox_hap_link`
- ZED2i optical: `zed2_left_camera_optical_frame`, `zed2_right_camera_optical_frame`
- IMU: `imu_link`; ZED IMU: `zed2_imu_link`
- GPS: `gps_link

## Notes

- Physics: DART engine, 500 Hz (`<max_step_size>0.002</max_step_size>`,
  `<real_time_update_rate>500</real_time_update_rate>`).
- The tread plates are animated by the custom `vl400-track-visual-system`
  gz-sim plugin; the Livox HAP scan pattern is generated by the custom
  `livox_laser_simulation` plugin (`src/`, `scan_patterns/HAP.csv`).
- The track belt uses anisotropic friction (low lateral `mu1`, high
  longitudinal `mu2`) so the robot skid-steers like a real tracked
  vehicle; `/odom_tracked` reports the resulting slip.
- The ZED2i depth camera publishes depth + point cloud (no color image);
  the left color image is `/zed2/left_raw/image_raw_color`.
