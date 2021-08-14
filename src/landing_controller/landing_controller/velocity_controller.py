import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node

from time import sleep

from std_msgs.msg import Header, Float32
from geometry_msgs.msg import TransformStamped, Twist, Vector3, Transform, Quaternion

from px4_msgs.msg import VehicleAttitude

from tf2_ros import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster

from landing_controller.velocity_commander import VelocityCommander
from landing_controller.vector_math import *


class VelocityController(Node):

    # namings
    TOPICS = {
        "P_GAIN" : "/landing_controller/p_gain"
    }
    TOPICS["CMD_VEL"] = VelocityCommander.TOPICS["CMD_VEL"]

    def __init__(self):
        super().__init__('velocity_controller')

        # parameters
        # static transformation from pixhawk to camera frame
        TF_PIX_TO_CAMERA = Transform(
            Vector3(z=.1),      # Translation from Pixhawk to Camera
            Quaternion()        # Rotation from Pixhawk to Camera
        )
        self.gains = {"P": 1.} # Initial controller gains

        # class variables
        self.aruco_timeout_reset = True

        # set up transform listener
        self.tf_buf = Buffer()
        self.tf_listener  = TransformListener(self.tf_buf, self, spin_thread=False)

        # /map frame (https://www.ros.org/reps/rep-0103.html#axis-orientation) 
        # to North-East-Down frame
        self.init_static_frame('map', 'ned', Vector3(),
                Quaternion(x=1., w=0.)) # Rotation of 180 degrees around x-axis

        # pixhawk to camera -> dependent on drone construction
        self.init_static_frame('pixhawk', 'camera', 
                self.TF_PIX_TO_CAMERA.Translation, self.TF_PIX_TO_CAMERA.Rotation) 

        # set up transform broadcaster (for debug)
        self.tf_broadcaster = TransformBroadcaster(self)

        # subscription callbacks
        self.sub_att = self.create_subscription(
            VehicleAttitude,
            'VehicleAttitude_PubSubTopic',
            self.attitude_callback,
            10)
        self.sub_ctrl_gain = self.create_subscription(
            Float32,
            self.TOPICS["P_GAIN"],
            self.process_p_gain_callback,
            1)

        # init publications
        self.vel_publisher = self.create_publisher(Twist, self.TOPICS["CMD_VEL"], 1)
            
        # timer callbacks
        self.control_timer = self.create_timer(.1, self.control)



    def attitude_callback(self, msg: VehicleAttitude):
        # rotation in msg is rotation from FRONT-RIGHT-DOWN (FRD) to
        # NORTH-EAST-DOWN (NED)
        # (see https://github.com/PX4/PX4-Autopilot/blob/master/msg/vehicle_attitude.msg)

        tf = Transform(rotation=numpy_ndarray_to_Quaternion(msg.q))
        tf_header = Header(stamp=self.get_clock().now().to_msg(), frame_id='ned')
        tf_stamped = TransformStamped(transform=tf, header=tf_header, child_frame_id='pixhawk')

        self.tf_buf.set_transform(tf_stamped, 'default_authority')

        self.publish_debug_frames()

    def process_p_gain_callback(self, msg: Float32):
        self.gains["P"] = msg.data


    def init_static_frame(self, source: str, target: str, trans: Vector3, rot: Quaternion):
        tf = Transform(translation=trans, rotation=rot)
        tf_header = Header(stamp=self.get_clock().now().to_msg(), frame_id=source)
        tf_stamped = TransformStamped(transform=tf, header=tf_header, child_frame_id=target)

        self.tf_buf.set_transform_static(tf_stamped, 'default_authority')



    def control(self):
        """Control algorithm to land drone

        :param Vector3 e: position error between pixhawk and aruco marker in ned frame
        """

        if self.tf_buf.can_transform('ned','aruco_marker', Time()):
            # determine control error
            e_tf = self.tf_buf.lookup_transform('ned','aruco_marker', Time())
            e = Vector3(x=e_tf.transform.translation.x, y=e_tf.transform.translation.y)

            # apply controller
            print(self.gains["P"])
            u = (e.x * self.gains["P"], e.y * self.gains["P"])
            print(e)
            print(u)

            self.pub_xy_vel(u)


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


    def publish_debug_frames(self):
        frames = ['ned', 'camera', 'pixhawk', 'aruco_marker']

        for frame in frames:
            if self.tf_buf.can_transform('map',frame, Time()):
                tf = self.tf_buf.lookup_transform('map',frame, Time())
                tf.header.stamp = self.get_clock().now().to_msg()
                tf.child_frame_id = frame + '_debug'
                
                self.tf_broadcaster.sendTransform(tf)



def numpy_ndarray_to_Quaternion(ndarr):
    Q = Quaternion()
    Q.w = ndarr[0].astype(float)
    Q.x = ndarr[1].astype(float)
    Q.y = ndarr[2].astype(float)
    Q.z = ndarr[3].astype(float)
    return Q


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
