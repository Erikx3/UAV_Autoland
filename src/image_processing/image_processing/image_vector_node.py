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
from geometry_msgs.msg import Transform, Vector3, Quaternion

import numpy as np
import cv2
from cv2 import aruco
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation


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
        # self.marker_length_dict = {"1": 0.063,
        #                            "3": 0.063,
        #                            "5": 0.031,
        #                            "7": 0.063,
        #                            "9": 0.063}  # Dictionary for all 5 marker lengths for DINA4

        self.marker_length_dict = {"1": 0.1635,
                                   "3": 0.1635,
                                   "4": 0.1635,
                                   "5": 0.082,
                                   "7": 0.408,
                                   "9": 0.1635}  # Dictionary for all 5 marker lengths for DINA2

        # Create publisher
        self.publisher_ = self.create_publisher(Transform, 'image_vectors', 10)

        # Some flags for logging
        self.camera_info_lis_callback_flag = True
        self.image_lis_callback_flag = True

    def camera_info_lis_callback(self, msg):
        self.camera_distortion = np.array(msg.d)
        self.camera_matrix = np.array(msg.k).reshape([3, 3])
        if self.camera_info_lis_callback_flag:
            self.get_logger().info(
                'I received the camera info {} \n {}'.format(str(self.camera_distortion), str(self.camera_matrix)))
            self.camera_info_lis_callback_flag = False

    def image_lis_callback(self, msg):
        # self.rgb_image_data = msg.data

        self.rgb_image_cv = CvBridge().imgmsg_to_cv2(msg, desired_encoding="rgb8")
        gray = cv2.cvtColor(self.rgb_image_cv, cv2.COLOR_BGR2GRAY)

        # self.get_logger().info(str([str(gray), str(self.aruco_dict) , self.aruco_parameters]))
        # self.get_logger().info('I received the image: {}'.format(str(self.rgb_image_cv)))
        # self.get_logger().info('I received the image from time {}'.format(str(msg.header.stamp)))
        rvec = None
        tvec = None
        corners, ids, rej = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_parameters)
        if ids is not None:
            self.get_logger().info("I found {} aruco markers!".format(len(ids)))
            for i, idt in enumerate(ids.flatten()):
                marker_length = self.marker_length_dict[str(idt)]
                # Check if matrix parameters are available
                if self.camera_matrix is not None and self.camera_distortion is not None:
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners=corners[i],
                                                                                   markerLength=marker_length,
                                                                                   cameraMatrix=self.camera_matrix,
                                                                                   distCoeffs=self.camera_distortion
                                                                                   )
                    rvec = rvec.flatten()
                    tvec = tvec.flatten()
                    # Always choose the one in the middle, else just pick one by random (or here last one)
                    if idt == 5:
                        break
	
        #check if available and if z transformation is positive (sometime local coordinate systems are flipped) 
        if tvec is not None and rvec is not None:
        	
        	if tvec[2]>0:
		        # Calculate quaternion from rotation vector
		        r = Rotation.from_rotvec(rvec)
		        quat = r.as_quat()  # These are automatically normalized

		        # Build ros geometry transform Message Output and publish it
		        T = Transform()
		        V = Vector3()
		        Q = Quaternion()
		        V.x = tvec[0]
		        V.y = tvec[1]
		        V.z = tvec[2]
		        Q.x = quat[0]
		        Q.y = quat[1]
		        Q.z = quat[2]
		        Q.w = quat[3]
		        T.translation = V
		        T.rotation = Q

		        self.publisher_.publish(T)
		        self.get_logger().info('I publish: ' + str(T))


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
