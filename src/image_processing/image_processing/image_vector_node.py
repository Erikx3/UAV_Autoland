# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Transform, Vector3, Quaternion

import message_filters
import numpy as np
import cv2
from cv2 import aruco
from cv_bridge import CvBridge


class ImageVectors(Node):

    def __init__(self):
        super().__init__('image_vector_node')
        # Listening to image topic
        self.image_sub = self.create_subscription(
            Image,
            'image_raw',
            self.image_lis_callback,
            1)
        self.image_sub  # prevent unused variable warning
        
        # Listening to camera info (should be static, for calibration parameters)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera_info',
            self.camera_info_lis_callback,
            1)
        self.camera_info_sub  # prevent unused variable warning
        
        # Variable for saving active image data and camera info
        self.rgb_image_data = None
        self.rgb_image_cv = None
        self.camera_distortion = None
        self.camera_matrix = None
        self.trans_vec = None
        self.rot_vec = None
        self.quat_vec = None
        
        # Init for Aruco markers
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_parameters = aruco.DetectorParameters_create()  # Initialize Parameters
        
        # Create publisher
        self.publisher_ = self.create_publisher(Transform, 'image_vectors', 10)
        	
    def camera_info_lis_callback(self, msg):
        self.camera_distortion = np.array(msg.d)
        self.camera_matrix = np.array(msg.k).reshape([3, 3])
        
        #self.get_logger().info('I received the camera info {} \n {}'.format(str(self.camera_distortion), str(self.camera_matrix)))
        
    def image_lis_callback(self, msg):
        #self.rgb_image_data = msg.data
        
        self.rgb_image_cv = CvBridge().imgmsg_to_cv2(msg, desired_encoding="rgb8")
        gray = cv2.cvtColor(self.rgb_image_cv, cv2.COLOR_BGR2GRAY)
        
        
        #self.get_logger().info(str([str(gray), str(self.aruco_dict) , self.aruco_parameters]))  
        #self.get_logger().info('I received the image: {}'.format(str(self.rgb_image_cv)))
        
        
        corners, ids, rej = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_parameters)
        if ids is not None:
        	self.get_logger().info("I found {} aruco markers!".format(len(ids)))       		
        
        
        #self.get_logger().info('I received the image from time {}'.format(str(msg.header.stamp)))
        
        
        T = Transform()
        V = Vector3()
        Q = Quaternion()
        V.x = 1.1
        V.y = 2.2
        V.z = 3.3
        Q.x = 0.1
        Q.y = 0.2
        Q.z = 0.3
        Q.w = 0.4
        T.translation = V
        T.rotation = Q
                
        self.publisher_.publish(T)
        #self.get_logger().info('I publish: ' + str(T))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ImageVectors()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
