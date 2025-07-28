from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('mycobot320_description'),
        'urdf',
        'robots',
        'mycobot320.urdf.xacro'
    )

    doc = xacro.parse(open(urdf_path))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()

    rviz_path = os.path.join(
        get_package_share_directory('mycobot320_description'),
        'rviz',
        'mycobot320.rviz'
    )

    return LaunchDescription([
        Node(
            package='mycobot320_analysis',
            executable='analyze_singularity_elbow',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path]
        )
    ])
