import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Transform, TransformStamped, Quaternion
from tf2_msgs.msg import TFMessage

from px4_msgs.msg import VehicleAttitude

class OrientationPublisher(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.sub_att = self.create_subscription(
            VehicleAttitude,
            'VehicleAttitude_PubSubTopic',
            self.process_att_callback,
            10)

        self.pub_tf2 = self.create_publisher(TFMessage, "/tf", 1)


    def process_att_callback(self, msg):
        # rotation in msg is rotation from FRONT-REAR-DOWN (FRD) to
        # NORTH-EAST-DOWN (NED)
        # (see https://github.com/PX4/PX4-Autopilot/blob/master/msg/vehicle_attitude.msg)

        T = Transform()

        T.rotation = numpy_ndarray_to_Quaternion(msg.q)

        TF2 = TFMessage()
        Tstamped = TransformStamped()
        Theader = Header()

        Theader.stamp = self.get_clock().now().to_msg()
        Theader.frame_id = "ned"

        Tstamped.transform = T
        Tstamped.child_frame_id = "pixhawk"
        Tstamped.header = Theader

        TF2.transforms = [Tstamped]
        self.pub_tf2.publish(TF2)


    def rot_x_180d(Q: Quaternion):
        """Returns rotation of quaternion of 180 degree around x-axis
        """

        q_x_180d = Quaternion()
        q_x_180d.x = 1.0
        q_x_180d.w = 0.0

        Q_new = Quaternion
        Q
        Q_new =  OrientationPublisher.quat_mult(Q, q_x_180d)

        return Q_new

    def quat_mult(q1: Quaternion, q2: Quaternion):
        """Returns quaternion product of q1 and q2
        """
        # that should actually be integrated in ROS but I couldn't find it
        Q = Quaternion()

        Q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z
        Q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y
        Q.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x
        Q.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w

        return Q



def main(args=None):
    rclpy.init(args=args)

    orient_pub = OrientationPublisher()

    rclpy.spin(orient_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    orient_pub.destroy_node()
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
