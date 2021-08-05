import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Header
from geometry_msgs.msg import Transform, TransformStamped, Quaternion, Vector3
from tf2_msgs.msg import TFMessage

from px4_msgs.msg import VehicleLocalPosition

from tf2_ros import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster

class TestArucoPublisher(Node):
    """
    This node is used to publish artificial aruco markers to test the control
    algorithm without the image processing node.
    """

    def __init__(self):
        super().__init__('aruco_test_publisher')

        # parameters
        self.OMEGA = .1
        self.F = 10.

        # inits
        self.angle = 0.

        # set up transform listener
        self.tf_buf = Buffer()
        self.tf_listener  = TransformListener(self.tf_buf, self, spin_thread=False)

        # define position of aruco marker
        #self.init_static_frame('world', 'aruco_marker_internal', 
        #        Vector3(),
        #        Quaternion(x=1.,w=0.)) # z-axis of markers points upwards

        # set up transform broadcaster (for debug)
        self.tf_broadcaster = TransformBroadcaster(self)

        # set up transform broadcaster (for debug)
        self.tf_broadcaster = TransformBroadcaster(self)

        # subscription callbacks
        self.sub_att = self.create_subscription(
            VehicleLocalPosition,
            'VehicleLocalPosition_PubSubTopic',
            self.position_callback,
            10)

        # set up timers
        self.control_timer = self.create_timer(1/self.F, self.update_marker)


    def position_callback(self, msg):
        trans = Vector3(x=msg.x, y=msg.y, z=msg.z)
        tf = Transform(translation=trans)
        tf_header = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
        tf_stamped = TransformStamped(transform=tf, header=tf_header, child_frame_id='ned')

        self.tf_buf.set_transform(tf_stamped, 'default_authority')

        self.publish_aruco_frame()


    def init_static_frame(self, source: str, target: str, trans: Vector3, rot: Quaternion):
        tf = Transform(translation=trans, rotation=rot)
        tf_header = Header(stamp=self.get_clock().now().to_msg(), frame_id=source)
        tf_stamped = TransformStamped(transform=tf, header=tf_header, child_frame_id=target)

        self.tf_buf.set_transform_static(tf_stamped, 'default_authority')


    def set_marker(self, p: Vector3):
        tf = Transform(translation=p)
        tf_header = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
        tf_stamped = TransformStamped(transform=tf, header=tf_header, child_frame_id='aruco_marker_internal')

        self.tf_buf.set_transform_static(tf_stamped, 'default_authority')


    def publish_aruco_frame(self):
        if self.tf_buf.can_transform('ned', 'aruco_marker_internal', Time()):
            tf = self.tf_buf.lookup_transform('ned', 'aruco_marker_internal', Time())

            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = 'ned_debug'
            tf.child_frame_id = 'aruco_marker'
            
            self.tf_broadcaster.sendTransform(tf)

    
    def update_marker(self):
        self.angle = self.angle + self.OMEGA/self.F

        R = 5
        marker = Vector3(
            x = R*math.sin(self.angle),
            y = R*math.cos(self.angle),
            z = 0.
            )

        self.set_marker(marker)




def main(args=None):
    rclpy.init(args=args)

    node = TestArucoPublisher()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
