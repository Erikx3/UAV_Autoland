import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Transform, TransformStamped, Vector3, Quaternion
from tf2_msgs.msg import TFMessage

from px4_msgs.msg import VehicleAttitude

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.sub_att = self.create_subscription(
            VehicleAttitude,
            'VehicleAttitude_PubSubTopic',
            self.process_att_callback,
            10)

        self.pub_tf2 = self.create_publisher(TFMessage, "/tf", 1)


    def process_att_callback(self, msg):
        T = Transform()

        T.rotation = numpy_ndarray_to_Quaternion(msg.q)

        TF2 = TFMessage()
        Tstamped = TransformStamped()
        Theader = Header()

        Theader.stamp = self.get_clock().now().to_msg()
        Theader.frame_id = "map"

        Tstamped.transform = T
        Tstamped.child_frame_id = "pixhawk"
        Tstamped.header = Theader

        TF2.transforms = [Tstamped]
        self.pub_tf2.publish(TF2)



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

def numpy_ndarray_to_Quaternion(ndarr):
    Q = Quaternion()
    Q.w = ndarr[0].astype(float)
    Q.x = ndarr[1].astype(float)
    Q.y = ndarr[2].astype(float)
    Q.z = ndarr[3].astype(float)
    return Q


if __name__ == '__main__':
    main()
