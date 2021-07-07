import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # RTPS connection with gazebo
        launch.actions.ExecuteProcess(
        	cmd=['micrortps_agent', 'start', '-t', 'UDP'],
        	output='screen'
        	),

        # Gazebo and PX4 Firmware
        # launch.actions.ExecuteProcess(
        # 	cmd=['make', 'px4_sitl_rtps', 'gazebo', 
        #         '-C', '~/PX4-Autopilot_1.12'],
        # 	output='screen'
        # 	),

        # Command to move drone
        # launch_ros.actions.Node(
        #     package='teleop_twist_keyboard',
        #     namespace='',
        #     executable='teleop_twist_keyboard',
        #     output='screen',
        #     name=['teleop']),

        # call general launch file (This could also be done nicer somehow)
        launch.actions.ExecuteProcess(
        	cmd=['ros2', 'launch', 'landing_controller', 'general.launch.py'],
        	output='screen'
        	),

    ])
