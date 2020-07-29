import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'arduino',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Launch Arduino node'),
        launch_ros.actions.Node(
            package='ros_arduino_python', executable='arduino_node.py', output='screen',
            name=[launch.substitutions.LaunchConfiguration('arduino'), 'arduino']),
    ])