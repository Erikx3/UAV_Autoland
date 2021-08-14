import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        # Image Processing Nodes
        launch_ros.actions.Node(
            package='image_processing',
            namespace='',
            executable='image_vector_node',
            output='screen',
            name=['image_vectore_name']),
        launch_ros.actions.Node(
            package='v4l2_camera',
            namespace='',
            executable='v4l2_camera_node',
            output='screen',
            name=['camera_node'])
            
        # launch.actions.ExecuteProcess(
        # 	cmd=['ros2', 'bag', 'record', '/image_raw', '/image_vectors'],
        # 	output='screen'
        # 	)
    ])
