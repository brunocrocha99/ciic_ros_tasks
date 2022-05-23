import os
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    task_name = LaunchConfiguration('namespace')

    task_name_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='control_car_task'
    )

    pkg_task = get_package_share_directory('ciic_ros_tasks')

    path_to_my_config = os.path.join(pkg_task, 'config', 'params_task_car.yaml')

    task_node = Node(
        parameters=[path_to_my_config],
        package='ciic_ros_tasks',
        namespace=task_name,
        executable='task_car_teleop'
    )

    return LaunchDescription([
        task_name_launch_arg,        
        task_node
    ])