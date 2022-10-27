import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
import numpy as np
import tf_transformations

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from ros2_aruco_interfaces.msg import ArucoMarkers

class ArucoNode(Node):
	def __init__(self):
		super().__init__('aruco_node')

		self.declare_parameter("marker_size", 0.14)	# Em metros
		self.declare_parameter("aruco_dict", "DICT_5X5_1000")
		self.declare_parameter("image_topic", "/camera/image_raw")

		self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
		aruco_dict = self.get_parameter("aruco_dict").get_parameter_value().string_value
		aruco_dict_id = cv2.aruco.__getattribute__(aruco_dict)
		image_topic = self.get_parameter("image_topic").get_parameter_value().string_value

		self.cv_bridge = CvBridge()

		self.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_id)
		self.aruco_params = cv2.aruco.DetectorParameters_create()

		self.calibration = np.array([[570.99457492, 0.0, 325.64860801], [0.0, 434.016448, 234.74391119], [0.0, 0.0, 1.0]])
		self.distortion = np.array([[2.18105604, -8.03481164, -0.25181316, 0.21993668, 20.62316516]])

		self.create_subscription(Image, image_topic, self.image_callback, 10)
		self.pub_ = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)
		self.pub_img_ = self.create_publisher(Image, '/camera/image_aruco', 10)

	def image_callback(self, img_msg):
		cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)

		markers = ArucoMarkers()
		markers.header.stamp = img_msg.header.stamp

		(corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)

		if ids is not None:
			rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.calibration, self.distortion)

			for i, id in enumerate(ids):
				theta = np.linalg.norm(rvecs[i])
				n = rvecs[i]/theta
				q_ra = tf_transformations.quaternion_about_axis(theta, n[0])
				self.get_logger().info('ids: {}'.format(ids, id, i)) # remover apos teste
				pose = Pose()
				pose.position.x = tvecs[i][0][0]
				pose.position.y = tvecs[i][0][1]
				pose.position.z = tvecs[i][0][2]
				pose.orientation.x = q_ra[0]
				pose.orientation.y = q_ra[1]
				pose.orientation.z = q_ra[2]
				pose.orientation.w = q_ra[3]
				markers.poses.append(pose)
				markers.ids.append(id[i])

				cv2.aruco.drawDetectedMarkers(cv_image, corners)
				cv2.drawFrameAxes(cv_image, self.calibration, self.distortion, rvecs[i], tvecs[i], 0.01)

			for(markerCorner, markerID) in zip(corners, ids):
				(topLeft, _, _, _) = markerCorner.reshape((4,2))
				cv2.putText(cv_image, str(markerID), (int(topLeft[0]), int(topLeft[1]) - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

			self.pub_img_.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
			self.pub_.publish(markers)

def main(args=None):
	rclpy.init(args=args)
	aruco_node = ArucoNode()
	rclpy.spin(aruco_node)
	aruco_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
