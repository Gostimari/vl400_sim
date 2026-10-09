import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('vl400_sim')
    xacro_file = os.path.join(pkg_share, 'urdf', 'vl400.xacro')

    model = LaunchConfiguration('model')
    use_gui = LaunchConfiguration('gui')

    robot_description = ParameterValue(
        Command(['xacro ', model]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        output='screen',
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_gui),
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(use_gui),
        output='screen',
    )

    rviz2 = ExecuteProcess(
        cmd=['rviz2'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=xacro_file,
            description='URDF/xacro file to visualize',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Use joint_state_publisher_gui to move the joints',
        ),
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])
