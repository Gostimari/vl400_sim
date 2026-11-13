# vl400_sim (ROS 2)

Gazebo simulation of the **VL400** tracked mulcher
robot in a forest world. The robot is a skid-steer tracked
vehicle with animated tread plates, a front mulcher, a Livox HAP lidar,
a ZED2i stereo camera, an IMU/GPS/magnetometer and a VLP-16 lidar.

## Installation

### Docker (recommended)

This is the **recommended way** to run the simulation — everything comes
in a self-contained image and the host only needs Docker.

```bash
# Let the container's root user open windows on your display. Needed
# once per login session (X forgets it on reboot/relogin); without it
# the Gazebo GUI and the sensor rendering die with
# "Authorization required, but no authorization protocol specified".
xhost +local:root

Go to the docker/ folder and run:

docker compose up             # run, creates the container; Ctrl-C stops the simulation

docker compose down           # stop, deletes the container
```

#### Jazzy and Lyrical compatibility

The package builds on both distros unmodified. `CMakeLists.txt` detects
which gz-sim configuration the distro provides — the unsuffixed vendor
configs (`gz-sim`, `gz-math`, `gz-msgs`, `gz-transport`, `gz-plugin`) on
Lyrical, the versioned ones (`gz-sim8`, `gz-math7`, `gz-msgs10`,
`gz-transport13`, `gz-plugin2`, with the plugin register in its own
`gz-plugin2-register` config) on Jazzy — and maps the link targets
automatically, so switching distros needs no edits.

## Manual install

### Cloning this repository (git LFS)

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
```

Rebuild after any pull/checkout that touches the mesh files — `install/`
holds copies, and gz-sim loads the meshes from there, not from `src/`.

### Dependencies

All dependencies are declared in `package.xml`. The recommended way to
install them is `rosdep` (it reads `package.xml` directly, so this list
can't drift out of sync):

```bash
sudo apt install python3-rosdep
rosdep init      # only the first time
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Or install them explicitly (ROS 2):

```bash
sudo apt install \
  ros-$ROS_DISTRO-ament-cmake \
  ros-$ROS_DISTRO-rosidl-default-generators ros-$ROS_DISTRO-rosidl-default-runtime \
  ros-$ROS_DISTRO-std-msgs ros-$ROS_DISTRO-sensor-msgs ros-$ROS_DISTRO-geometry-msgs \
  ros-$ROS_DISTRO-nav-msgs ros-$ROS_DISTRO-rosgraph-msgs \
  ros-$ROS_DISTRO-rclcpp ros-$ROS_DISTRO-rclpy ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-ament-index-cpp \
  ros-$ROS_DISTRO-gz-sim-vendor ros-$ROS_DISTRO-gz-math-vendor \
  ros-$ROS_DISTRO-gz-msgs-vendor ros-$ROS_DISTRO-gz-transport-vendor \
  ros-$ROS_DISTRO-gz-plugin-vendor \
  ros-$ROS_DISTRO-ros-gz-sim ros-$ROS_DISTRO-ros-gz-bridge \
  ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-joint-state-publisher-gui \
  ros-$ROS_DISTRO-rviz2 \
  python3-scipy
```

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select vl400_sim
source install/setup.bash
```


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
| `x` `y` `z` | `4` `10` `5` | robot spawn pose override; see below |

The spawn pose is **automatic per world**: the launch looks up the world
file in a per-world table (`spawn_poses` in `gazebo.launch.py`) and spawns
the robot at a sensible height above the terrain at (4, 10). Passing
explicit `x:=`/`y:=`/`z:=` launch arguments overrides the table.

To add a new world (or move the spawn point), measure the terrain height
`h` at the desired `x, y` and add an entry with `z ≈ h + 0.35` to the
`spawn_poses` dict in `launch/gazebo.launch.py`.

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
  world:=/root/ros2_ws/src/vl400_sim/worlds/forest_ground-200-hard.world
```

## Fog generator

Each forest world includes a `model://Fog Generator` next to the robot spawn:
a gz-sim particle emitter that renders a wide, ~1 m high fog bank hugging
the ground (`models/Fog Generator/model.sdf`). The bank is pure visuals: no
collision, no lidar returns, and the robot drives through it. Key emitter
parameters (all in `model.sdf`):

| parameter | value | effect |
|---|---|---|
| `<size>` | `20 0.2 20` | emission slab: wide and 0.2 m thick (axes remapped by the particle renderer) |
| `<rate>` | `150` | puffs per second (density) |
| `<particle_size>` | `2.5 2.5 2.5` | puff size in m |
| `<min/max_velocity>` | `0.05` / `0.1` | slow upward drift |
| `<lifetime>` | `3` | caps the visible height (rise = velocity x lifetime) |

The `gz-sim-particle-emitter-system` plugin (loaded by every world) exposes
the emitter command topic and honors lifetime/velocity, so the fog can be
toggled and moved at runtime.

Activate / deactivate the fog:

```bash
gz topic -t /model/fog_generator/link/fog_link/particle_emitter/emitter/cmd \
  -m gz.msgs.ParticleEmitter -p 'emitting: {data: true}'
```

(`{data: false}` clears it within seconds.)

Launch the simulation with the fog in another position (all three
arguments are required; units are world coordinates, `z` should match the
terrain height at that spot):

```bash
ros2 launch vl400_sim gazebo.launch.py fog_x:=20 fog_y:=20 fog_z:=5.2
```

Move the fog while the simulation is running (note: this is a gz
*service*, not a topic, so it does not show in `gz topic -l`; replace
`forest_ground-50-easy` with the internal name of your world):

```bash
gz service -s /world/forest_ground-50-easy/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 300 \
  --req 'name: "fog_generator", position: {x: 20, y: 20, z: 4.3}, orientation: {w: 1.0}'
```

The default position is defined by the `<include><pose>` of the Fog
Generator in each `.world` file; without `fog_x/fog_y/fog_z` the fog
stays there.

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
- GPS: `gps_link`

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
