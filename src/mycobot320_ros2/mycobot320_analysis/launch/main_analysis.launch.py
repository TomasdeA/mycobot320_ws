from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_desc = get_package_share_directory('mycobot320_description')

    urdf_path = os.path.join(pkg_desc, 'urdf', 'robots', 'mycobot320.urdf.xacro')
    doc = xacro.process_file(urdf_path)
    robot_description_config = doc.toxml()

    rviz_path = os.path.join(pkg_desc, 'rviz', 'mycobot320.rviz')

    # Launch args
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='shoulder',
        description='Which analysis to run: shoulder, elbow, wrist, show'
    )

    with_rviz_arg = DeclareLaunchArgument(
        'with_rviz', default_value='true',
        description='Launch RViz (true/false)'
    )

    mode = LaunchConfiguration('mode')
    with_rviz = LaunchConfiguration('with_rviz')

    # Nodo del análisis dinámico (único que se ejecuta)
    shoulder_node = Node(
        package='mycobot320_analysis',
        executable='analyze_singularity_shoulder',
        name='shoulder_analysis',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'shoulder'"]))
    )

    elbow_node = Node(
        package='mycobot320_analysis',
        executable='analyze_singularity_elbow',
        name='elbow_analysis',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'elbow'"]))
    )

    wrist_node = Node(
        package='mycobot320_analysis',
        executable='analyze_singularity_wrist',
        name='wrist_analysis',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'wrist'"]))
    )

    show_node = Node(
        package='mycobot320_analysis',
        executable='show_configurations',
        name='show_configurations',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'show'"]))
    )

    # Lista de nodos posibles
    analysis_nodes = [shoulder_node, elbow_node, wrist_node, show_node]

    # Un shutdown handler por cada nodo
    shutdown_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[Shutdown()]
            )
        )
        for node in analysis_nodes
    ]

    return LaunchDescription([
        mode_arg,
        with_rviz_arg,

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
            arguments=['-d', rviz_path],
            condition=IfCondition(with_rviz)
        ),

        *analysis_nodes,
        *shutdown_handlers,
    ])
