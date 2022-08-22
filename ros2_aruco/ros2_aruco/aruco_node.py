import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
import numpy as np
import tf_transformations

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from ros2_aruco.utils import quaternion_from_euler, euler_from_rvec

class ArucoNode(Node):
	def __init__(self):
		super().__init__('aruco_node')

		self.declare_parameter("marker_size", .02)
		self.declare_parameter("aruco_dict", "DICT_ARUCO_ORIGINAL")
		self.declare_parameter("image_topic", "/camera/image_raw")

		self.marker_size = self.get_parameter(
			"marker_size").get_parameter_value().double_value
		aruco_dict = self.get_parameter(
			"aruco_dict").get_parameter_value().string_value
		aruco_dict_id = cv2.aruco.__getattribute__(aruco_dict)
		image_topic = self.get_parameter(
			"image_topic").get_parameter_value().string_value

		self.pub_ = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)
		self.cv_bridge = CvBridge()
		self.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_id)
		self.aruco_params = cv2.aruco.DetectorParameters_create()
		self.calibration = np.array([[584.32262583, 0.0, 253.41645929], [
									0.0, 574.60800033, 233.93778173], [0.0, 0.0, 1.0]])
		self.distortion = np.array(
			[[0.33706852, -1.65778338, -0.00908046, -0.04888188, 3.89528563]])

		self.create_subscription(Image, image_topic, self.image_callback, 10)

	def image_callback(self, img_msg):
		cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)

		markers = ArucoMarkers()
		markers.header.stamp = img_msg.header.stamp

		(corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)

		if ids is not None:
			rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.calibration, self.distortion)

			for i, id in enumerate(ids):
				pose = Pose()
				pose.position.x = tvecs[i][0][0]
				pose.position.y = tvecs[i][0][1]
				pose.position.z = tvecs[i][0][2]
				self.get_logger().info('Publishing: "%s"' % ('id: %d' % id))
				(row, pitch, yaw) = tf_transformations.euler_from_matrix(cv2.Rodrigues(rvecs[i])[0])

				quaternion = tf_transformations.quaternion_from_euler(row, pitch, yaw)
				pose.orientation.x = quaternion[0]
				pose.orientation.y = quaternion[1]
				pose.orientation.z = quaternion[2]
				pose.orientation.w = quaternion[3]

				markers.poses.append(pose)
				markers.ids.append(id[0])
			self.get_logger().info('Publishing: "%s"' %
								   ('Pose - x: {} y: {} z: {}').format(pose.position.x, pose.position.y, pose.position.z))
			self.pub_.publish(markers)

def main(args=None):
	rclpy.init(args=args)
	aruco_node = ArucoNode()
	rclpy.spin(aruco_node)
	aruco_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
