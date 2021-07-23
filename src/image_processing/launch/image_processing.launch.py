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
            name=['camera_node']),
        
        # Transformation publishers 
        launch_ros.actions.Node(
            package='image_processing',
            namespace='',
            executable='orientation_publisher',
            output='screen',
            name=['tf2_pub_orientation_pixhawk']),
        # /map frame (https://www.ros.org/reps/rep-0103.html#axis-orientation) 
        # to North-East-Down frame
        launch_ros.actions.Node(
            package='tf2_ros',
            namespace='',
            executable='static_transform_publisher',
            output='screen',
            name=['tf2_pub_orientation_NED'],
            arguments=["0","0","0",
                "0.","0.","3.141593",
                "/map", "/ned"]),
        launch_ros.actions.Node(
            package='tf2_ros',
            namespace='',
            executable='static_transform_publisher',
            output='screen',
            name=['tf2_pub_orientation_camera'],
            arguments=["0","0",".1",    # pos of camera relative to pixhawk
                "0","0","0",            # rotation (yaw, pitch, roll) of camera relative to pixhawk (should be zero)
                "/pixhawk", "/camera"]),
        
        # ROS bag
        launch.actions.ExecuteProcess(
        	cmd=['ros2', 'bag', 'record', '/image_raw', '/image_vectors'],
        	output='screen'
        	)
    ])
