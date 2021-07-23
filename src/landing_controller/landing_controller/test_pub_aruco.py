import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Transform, TransformStamped, Quaternion
from tf2_msgs.msg import TFMessage

from px4_msgs.msg import VehicleAttitude

class TestArucoPublisher(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.pub_tf2 = self.create_publisher(TFMessage, "/tf", 1)

        self.pub_tf_timer = self.create_timer(.1, self.pub_tf)

    def pub_tf(self):
        tfm = TFMessage()
        tf = TransformStamped()

        tf.header.frame_id = "camera"
        tf.child_frame_id = "aruco_marker"
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.transform.translation.x = 1.

        tfm.transforms = [tf]

        self.pub_tf2.publish(tfm)



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
