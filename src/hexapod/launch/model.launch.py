import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    robot_xacro_name = "hexapod_robot"
    package_name = "hexapod"
    model_file_relative_path = "model/robot.xacro"
    rviz_file_relative_path = "rviz/config.rviz"

    path_model_file = os.path.join(
        get_package_share_directory(package_name), 
        model_file_relative_path
        )
    
    robot_description = xacro.process_file(path_model_file).toxml()

    path_rviz_config = os.path.join(
        get_package_share_directory(package_name),
        rviz_file_relative_path
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}]
    )

    node_joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        parameters=[{"robot_description": robot_description}]
    )

    launch_rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", path_rviz_config],
        output="screen"
    )

    launch_description = LaunchDescription()

    launch_description.add_action(node_robot_state_publisher)
    launch_description.add_action(node_joint_state_publisher)
    launch_description.add_action(launch_rviz2)

    return launch_description
