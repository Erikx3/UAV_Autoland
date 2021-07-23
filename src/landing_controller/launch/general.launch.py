import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from landing_controller.velocity_commander import VelocityCommander

###
### This launch file is not supposed to be run manually. It is 
### included by other launchfiles
###


def generate_launch_description():
    # prepare launch command by inserting all necessary topics
    launch_cmd = ['ros2', 'bag', 'record']
    for topic in VelocityCommander.TOPICS:
        launch_cmd.append(VelocityCommander.TOPICS[topic])
    # Image Topics
    launch_cmd.append('/image_raw')  # Image topic from camera
    launch_cmd.append('/image_vectors')  # Image Vector for transformation


    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='landing_controller',
            namespace='',
            executable='velocity_commander',
            output='screen',
            name=['velocity_commander']),

        launch.actions.ExecuteProcess(
        	cmd=launch_cmd,
        	output='screen'
        	)
    ])
