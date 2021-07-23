import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

import math

from std_msgs.msg import Header, Float32
from geometry_msgs.msg import TransformStamped, Twist, Vector3, Transform
from tf2_msgs.msg import TFMessage
from tf2_ros import BufferInterface

from landing_controller.velocity_commander import VelocityCommander
from tf2_ros.transform_listener import TransformListener


class VelocityController(Node):

    # parameters
    TOPICS = {
        "P_GAIN" : "/landing_controller/p_gain"
    }
    TOPICS["CMD_VEL"] = VelocityCommander.TOPICS["CMD_VEL"]

    FRAMES = {
        "ARUCO"   : "aruco_marker",
        "NED"           : "ned"
    }

    # time in seconds in which the timeout function is called
    # timeout is earlist executed after ARUCO_TIMEOUT seconds
    # and latest after 2*ARUCO_TIMEOUT seconds
    ARUCO_TIMEOUT = .5   


    def __init__(self):
        super().__init__('velocity_controller')

        # class variables
        self.aruco_timeout_reset = True
        self.tf_buf = BufferInterface()
        self.gains = {"P", 0.} # Initial controller gains

        # subscription callbacks
        self.sub_tf = self.create_subscription(
            TFMessage,
            "/tf",
            self.process_tf_callback,
            1)
        self.sub_ctrl_gain = self.create_subscription(
            Float32,
            self.TOPICS["P_GAIN"],
            self.process_p_gain_callback,
            1)

        # init publications
        self.vel_publisher = self.create_publisher(Twist, self.TOPICS["CMD_VEL"], 1)
            
        # timer callbacks
        self.aruco_timout_timer = self.create_timer(self.ARUCO_TIMEOUT, self.aruco_timer_callback)

        # transform listeners
        #self.tf_listener = TransformListener(self.tf_buf, self, spin_thread=True)


    def process_tf_callback(self, msg: Twist):
        """Applies control algorithm when new aruco frame was received

        :param Twist msg: received Twist message
        """
        print("Callback TF")
        for transformStamped in msg.transforms:
            print('Got child frame ' + transformStamped.child_frame_id)
            if transformStamped.child_frame_id == self.FRAMES["ARUCO"]:
                self.aruco_timeout_reset = False

                tf = self.tf_buf.lookup_transform(self.FRAMES["NED"], self.FRAMES["ARUCO"], 0)
                print(tf)

                #e = -self.tf_buf.transform(Vector3(), self.FRAMES["NED"], Duration(seconds=.1))
                #self.control(e)

                return


    def control(self, e: Vector3):
        """Control algorithm to land drone

        :param Vector3 e: position error between pixhawk and aruco marker in ned frame
        """

        # currently, only hovering over the aruco marker is implemented
        u = (e.x, e.y) * self.gains["P"]

        self.pub_xy_vel(e)
        

    def aruco_timer_callback(self):
        """Checks aruco_timeout_variable and executes timeout if it was not set
        since the last call
        """

        if self.aruco_timeout_reset == True:
            self.timeout()

        self.aruco_timeout_reset = True


    def timeout(self):
        self.pub_xy_vel((0.,0.))


    def pub_xy_vel(self, v: tuple):
        """Publish twist message on TOPICS["CMD_VEL"] where the two entries of
        v correspond to the commanded x- and y velocity.

        :param tuple v: x and y velocity to be commanded to pixhawk
        """

        if len(v) != 2:
            self.get_logger().info("Wrong number of entries in v input of send_xy_vel.")
            v = (0.,0.)

        T = Twist()
        T.linear.x = v[0]
        T.linear.y = v[1]

        self.vel_publisher.publish(T)


    def process_p_gain_callback(self, msg: Float32):
        self.gains["P"] = msg

"""
class TfLine:
    def __init__(self, node: Node, line: tuple) -> None:
        self.node = node
        self.line = line
        self.transforms = [Transform()] * len(line-1)

        node.create_subscription(TFMessage, "/tf", self.callback, 5)
        node.create_subscription(TFMessage, "/tf_static", self.callback, 5)


    def callback(self, msg: TFMessage):
        for transform in msg.transforms:
            self.add_transform(transform)


    def add_transform(self, tf: TransformStamped):
        p_frame = tf.header.frame_id    # parent frame
        c_frame = tf.child_frame_id     # child frame

        if p_frame in self.line and c_frame in self.line:
            p_index = self.line.index(p_frame)
            c_index = self.line.index(c_frame)

            # if c_frame comes after p_frame, refresh transformation
            if p_frame + 1 == c_frame:
                self.transforms[p_index] = tf.transform

            # if c_frame comes before p_frame, invert transformation
            elif p_frame - 1 == c_frame:
                self.transforms[p_index] = TfLine.invert_transform(tf.transform)

    def get_transform(self):
        tf = Transform()

        for transform in self.transforms:
            tf.translation.x = tf.translation.x + transform.translation.x


    def invert_transform(tf: Transform):
        tf.rotation.w = - tf.rotation.w
        tf.translation.x = - tf.translation.x
        tf.translation.y = - tf.translation.y
        tf.translation.z = - tf.translation.z

        return tf
"""



        





def main(args=None):
    rclpy.init(args=args)

    vel_ctrl = VelocityController()

    rclpy.spin(vel_ctrl)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vel_ctrl.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
