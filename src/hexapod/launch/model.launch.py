import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_ros
from launch_ros.actions import Node
import xacro

robot_xacro_name = "hexapod_robot"
package_name = "hexapod"
model_file_relative_path = "model/robot.xacro"
rviz_file_relative_path = "rviz/config.rviz"
world_relative_path = "worlds/empty.sdf"
ros2_control_relative_path = "config/robot_controller.yaml"
bridge_params_relative_path = "config/bridge_parameters.yaml"

def generate_launch_description():
    path_model_file = os.path.join(
        get_package_share_directory(package_name), 
        model_file_relative_path
        )
    
    robot_description = xacro.process_file(path_model_file).toxml()

    path_rviz_config = os.path.join(
        get_package_share_directory(package_name),
        rviz_file_relative_path
    )

    path_ros2_control = os.path.join(
        get_package_share_directory(package_name),
        ros2_control_relative_path
    )

    world_path = os.path.join(
        get_package_share_directory(package_name),
        world_relative_path
    )

    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        bridge_params_relative_path
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", f"-r -v4 {world_path}")]
    )

    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]"],
        output="screen"
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "robot_system_position",
            "-allow_renaming", "true"
        ]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}]
    )

    gazebo_ros_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}"
        ],
        output="screen"
    )

    launch_rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", path_rviz_config],
        output="screen"
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["legs_controller", "--param-file", path_ros2_control]
    )

    launch_description = LaunchDescription()

    launch_description.add_action(gazebo)
    launch_description.add_action(gazebo_bridge)
    launch_description.add_action(gz_spawn_entity)
    launch_description.add_action(node_robot_state_publisher)
    launch_description.add_action(gazebo_ros_bridge_node)
    launch_description.add_action(launch_rviz2)
    launch_description.add_action(joint_state_broadcaster)
    launch_description.add_action(robot_controller_spawner)

    return launch_description
