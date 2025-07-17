from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('mycobot320_description')

    # Paths
    default_model_path = PathJoinSubstitution([
        pkg_share, 'urdf', 'robots', 'mycobot320.urdf.xacro'
    ])
    default_rviz_config_path = PathJoinSubstitution([
        pkg_share, 'rviz', 'mycobot320.rviz'
    ])

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(name='use_gui', default_value='true', choices=['true', 'false'],
                              description='Use joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                              description='Absolute path to robot URDF/XACRO file'),
        DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                              description='Absolute path to rviz config file'),

        # Joint State Publisher (GUI or not)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(LaunchConfiguration('use_gui'))
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('use_gui'))
        ),

        # Robot State Publisher (from xacro)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')])
            }],
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')]
        )
    ])
