import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'excavator_pid_controller'
    #path to your parameter file
    params_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'pid_params.yaml'
    )

    pid_controller_node = Node(
        package=package_name,
        executable='pid_node',
        name='excavator_pid_controller_node',
        output='screen',
        parameters=[params_file_path]
    )

    return LaunchDescription([
        pid_controller_node
    ])