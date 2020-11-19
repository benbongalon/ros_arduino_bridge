from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arduino',
            default_value=[EnvironmentVariable('USER'), '_'],
            description='Launch Arduino node'),
        Node(
            package='ros_arduino_python', 
            executable='arduino_node.py', 
            output='screen',
            name=[LaunchConfiguration('arduino'), 'arduino']),
    ])