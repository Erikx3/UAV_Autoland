import rclpy
from rclpy.node import Node

import math

from geometry_msgs.msg import Twist, Vector3
from px4_msgs.msg import Timesync, OffboardControlMode, TrajectorySetpoint, VehicleCommand

class VelocityCommander(Node):

    def __init__(self):
        super().__init__('velocity_commander')

        # class variables
        self.keep_alive = True
        self.key = -1
        self.t_sync = 0
        self.vel = Vector3()
        self.pub_counter = 0

        # subscription callbacks
        self.sub_timesync = self.create_subscription(
            Timesync,
            'Timesync_PubSubTopic',
            self.process_timesync_callback,
            1)
        self.sub_vel = self.create_subscription(
            Twist,
            'cmd_vel',
            self.process_vel_callback,
            1)


        # init publications
        self.offb_control_mode_publisher = self.create_publisher(OffboardControlMode, "/OffboardControlMode_PubSubTopic", 10)
        self.traj_sp_publisher = self.create_publisher(TrajectorySetpoint, "/TrajectorySetpoint_PubSubTopic", 10)
        self.veh_cmd_publisher = self.create_publisher(VehicleCommand, "/VehicleCommand_PubSubTopic", 10)

        self.pub_vel_timer = self.create_timer(.1, self.timer_callback)

        self.pub_offb_control_mode()

    def timer_callback(self):
        # change to offboard mode after 10 loops
        if self.pub_counter == 10:
            self.pub_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.arm()

            print("Arming")
            print(VehicleCommand.VEHICLE_CMD_DO_SET_MODE)

        self.pub_offb_control_mode()
        self.pub_trajectory_setpoint()

        # stop counter
        if self.pub_counter < 11:
            self.pub_counter = self.pub_counter + 1
        

    def pub_offb_control_mode(self):
        mode = OffboardControlMode()

        mode.timestamp = self.t_sync
        mode.position = True
        mode.velocity = True
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False

        self.offb_control_mode_publisher.publish(mode)



    def pub_trajectory_setpoint(self):
        # without timesync, the message will not be accepted
        if self.t_sync == 0:
            return

        sp = TrajectorySetpoint()

        sp.timestamp = self.t_sync

        sp.x = math.nan
        sp.y = math.nan
        sp.z = -3.

        sp.vx = self.vel.x
        sp.vy = self.vel.y
        sp.vz = self.vel.z

        self.traj_sp_publisher.publish(sp)


    def process_timesync_callback(self, msg):
        self.t_sync = msg.timestamp

    def process_vel_callback(self, msg):
        self.vel = msg.linear

    def arm(self):
        self.pub_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1., 0.)
        

    def pub_vehicle_command(self, command, param1, param2):
        cmd = VehicleCommand()

        cmd.timestamp = self.t_sync
        cmd.param1 = param1
        cmd.param2 = param2
        cmd.command = command
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True

        self.veh_cmd_publisher.publish(cmd)



def main(args=None):
    rclpy.init(args=args)

    vel_com = VelocityCommander()

    rclpy.spin(vel_com)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vel_com.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
