import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # RTPS connection with raspberry pi
        launch.actions.ExecuteProcess(
        	cmd=['micrortps_agent', 'start', '-t', 'UART', '-d', 'ttyUSB0'],
        	output='screen'
        	),

        # call general launch file (This could also be done nicer somehow)
        launch.actions.ExecuteProcess(
        	cmd=['ros2', 'launch', 'landing_controller', 'general.launch.py'],
        	output='screen'
        	),

    ])
