import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='example_python',
            namespace='example',
            executable='example_talker',
            output='screen',
            name=['talker']),
        launch_ros.actions.Node(
            package='example_python',
            namespace='example',
            executable='example_listener',
            output='screen',
            name=['listener']),
    ])
