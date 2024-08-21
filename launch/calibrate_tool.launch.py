import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    calibrate_tool_node = Node(
        package='force_torque_sensor',
        executable='calibrate_tool_node',
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time}
        ]
    )

    return [calibrate_tool_node]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true')
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])