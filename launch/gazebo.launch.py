import os
import re
import subprocess
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('vl400_sim')
    xacro_file = os.path.join(pkg_share, 'urdf', 'vl400.xacro')
    bridge_file = os.path.join(pkg_share, 'config', 'bridge.yaml')
    default_world = os.path.join(pkg_share, 'worlds', 'forest_ground-50-medium.world')

    # Let gz resolve package:// mesh URIs inside the URDF and
    # model:// URIs inside the world file.
    share_root = os.path.dirname(pkg_share)
    models_dir = os.path.join(pkg_share, 'models')
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(
        [share_root, models_dir, os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )

    # Custom gz-sim systems built by this package: the TrackVisual tread
    # plate animation (libvl400-track-visual-system.so) and the Livox
    # lidar simulation plugin (liblivox_laser_simulation.so).
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join(
        [
            os.path.join(get_package_prefix('vl400_sim'), 'lib'),
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        ]
    )

    world = LaunchConfiguration('world')
    gpu = LaunchConfiguration('gpu')
    organize_cloud = LaunchConfiguration('organize_cloud')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    headless = LaunchConfiguration('headless')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' gpu:=', gpu,
            ' organize_cloud:=', organize_cloud,
        ]),
        value_type=str,
    )

    # Gazebo (gz-sim) with the selected world, running on start.
    # headless:=true runs the server only ('-s') with offscreen
    # rendering for the sensors.
    gz_args = [
        world,
        ' -r ',
        IfElseSubstitution(headless, ' -s --headless-rendering', ''),
    ]
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # URDF -> robot_state_publisher (TF tree on the ROS side)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # Spawn the vl400 into the running world. The URDF (with the full
    # tread-link chain) is too large for the create node's -string
    # command line argument, so it is written to a file and spawned
    # with -file instead.
    def launch_spawn(context):
        gpu_arg = gpu.perform(context)
        organize_arg = organize_cloud.perform(context)
        urdf_out = os.path.join(tempfile.gettempdir(), 'vl400_spawn.urdf')
        with open(urdf_out, 'w') as f:
            subprocess.run(
                ['xacro', xacro_file,
                 'gpu:=' + gpu_arg, 'organize_cloud:=' + organize_arg],
                check=True, stdout=f)
        return [Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'vl400',
                '-file', urdf_out,
                '-x', x.perform(context),
                '-y', y.perform(context),
                '-z', z.perform(context),
            ],
            output='screen',
        )]

    spawn = OpaqueFunction(function=launch_spawn)

    # ROS <-> Gazebo transport bridge (see config/bridge.yaml).
    # The /joint_states bridge entry is world-scoped: gz publishes it as
    # /world/<world>/model/vl400/joint_state, where <world> is the
    # INTERNAL name of the world file (<world name="...">, which can
    # differ from the filename, e.g. robot.world is "default"). The
    # config file carries the default world's name; patch in the name of
    # the world actually being launched so /joint_states (and with it
    # the wheel/track TF on the ROS side) keeps flowing in every world.
    def launch_bridge(context):
        world_path = world.perform(context)
        world_name = 'forest_ground-100-easy'
        try:
            with open(world_path) as wf:
                match = re.search(r'<world\s+name="([^"]+)"', wf.read())
            if match:
                world_name = match.group(1)
        except OSError:
            pass
        with open(bridge_file) as bf:
            bridge_cfg = bf.read()
        bridge_cfg = bridge_cfg.replace(
            '/world/forest_ground-100-easy/model/vl400/joint_state',
            '/world/' + world_name + '/model/vl400/joint_state')
        bridge_out = os.path.join(tempfile.gettempdir(), 'vl400_bridge.yaml')
        with open(bridge_out, 'w') as f:
            f.write(bridge_cfg)
        return [Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_out, 'use_sim_time': True}],
            output='screen',
        )]

    bridge = OpaqueFunction(function=launch_bridge)

    magnetometer = Node(
        package='vl400_sim',
        executable='magnetometer_for_simulation.py',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gpu', default_value='true'),
        DeclareLaunchArgument('organize_cloud', default_value='false'),
        DeclareLaunchArgument('x', default_value='4'),
        DeclareLaunchArgument('y', default_value='10'),
        DeclareLaunchArgument('z', default_value='5'),
        DeclareLaunchArgument('headless', default_value='false'),
        robot_state_publisher,
        gz_sim,
        spawn,
        bridge,
        magnetometer,
    ])
