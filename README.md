# l400_description (ROS 2)

ROS 2 port of the ROS 1 `l400_description` package: the same tractor
model (meshes, links, joints), sensors (VLP-16 lidar, IMU,
magnetometer, GPS) and differential-drive simulation, now targeting
**ROS 2 Lyrical** with **Gazebo Jetty (gz-sim 10)** via `ros_gz`.

## Layout

```
urdf/realTractor_V2.xacro   tractor model with gz-sim systems
urdf/velodyne_vlp16.urdf.xacro  VLP-16 macro (gz gpu_lidar/lidar sensor)
worlds/robot.world, example.world  gz-sim worlds (self-contained)
launch/gazebo.launch.py      full simulation + bridge + helper nodes
launch/display.launch.py     rviz2 + joint_state_publisher_gui
config/bridge.yaml           ros_gz_bridge topic mappings
msg/CustomMessage.msg        ported custom message (lowercase fields)
scripts/                     rclpy ports of the ROS 1 helper nodes
meshes/                      unchanged mesh assets
```

## Build

```bash
sudo apt install ros-lyrical-ros-gz ros-lyrical-xacro \
     ros-lyrical-robot-state-publisher ros-lyrical-joint-state-publisher-gui \
     ros-lyrical-rviz2 python3-scipy
cd ~/ros2_ws
colcon build --packages-select l400_description
source install/setup.bash
```

## Run

```bash
# Full simulation (world, tractor, sensors, bridge)
ros2 launch l400_description gazebo.launch.py

# Headless (no GUI, offscreen sensor rendering)
ros2 launch l400_description gazebo.launch.py headless:=true

# Visualization only (rviz2, sliders to move joints)
ros2 launch l400_description display.launch.py
```

## ROS topics (bridged from Gazebo)

| ROS topic | Type | Source |
|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | ROS -> DiffDrive |
| `/odom` | nav_msgs/Odometry | DiffDrive odometry |
| `/joint_states` | sensor_msgs/JointState | wheel joints |
| `/imu` | sensor_msgs/Imu | IMU on `duro_link` |
| `/fix` | sensor_msgs/NavSatFix | GPS on `gps_link` |
| `/mag` | sensor_msgs/MagneticField | magnetometer |
| `/velodyne_points` | sensor_msgs/PointCloud2 | VLP-16 point cloud |
| `/velodyne_scan` | sensor_msgs/LaserScan | VLP-16 scan |
| `/clock` | rosgraph_msgs/Clock | sim time (use_sim_time) |

Helper nodes (same behaviour as ROS 1): `fake_state_publisher.py`
(`stop`), `pub_GPS_XY_trator.py` (`XY_coordinates`),
`magnetometer_for_simulation.py` (`heading`).

Drive the tractor:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

## Notes

- `gpu:=true` (default) uses the `gpu_lidar` sensor; `gpu:=false`
  selects the CPU `lidar` sensor type.
- The ROS 1 `velodyne_description` / `velodyne_gazebo_plugins` and
  `hector_gazebo` dependencies are no longer needed; the lidar is a
  native gz-sim sensor and GPS/compass come from gz's `navsat` and
  `magnetometer` sensors. Their noise parameters are not identical to
  the hector plugins.
- The ROS 1 `imu_complementary_filter` node from `spawn.launch` was
  not ported (no ROS 2 release of that package); `imu_filter_madgwick`
  is a drop-in alternative if you need filtered IMU output.
- The classic Gazebo per-wheel friction hack (`<surface><friction>`
  inside URDF collisions) was dropped; gz uses its own physics
  (DART) and that block was never valid URDF.
