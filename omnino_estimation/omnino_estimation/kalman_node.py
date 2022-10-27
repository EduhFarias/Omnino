import rclpy
from rclpy.node import Node

from omnino_estimation.kalman import Kalman
import tf_transformations
import numpy as np

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose

class KalmanNode(Node):
	def __init__(self):
		super().__init__("kalman_node")
		self.kalman = Kalman()
		self.aruco_msg = Pose()
		self.imu_msg = Imu()
		self.imu_msg.linear_acceleration.x = 0.0
		self.imu_msg.linear_acceleration.y = 0.0
		self.imu_msg.angular_velocity.z = 0.0

		self.imu_sub = self.create_subscription(Imu, "imu", self.imu_callback, 10)
		self.aruco_sub = self.create_subscription(Pose, "aruco_pose", self.aruco_callback, 10)
		self.pub_ = self.create_publisher(Pose, "estimated_pose", 10)

	def imu_callback(self, imu_msg):
		if not self.isStopped(imu_msg):
			self.imu_msg = imu_msg
			self.kalman_callback()

	def aruco_callback(self, aruco_msg):
		self.aruco_msg = aruco_msg

	def kalman_callback(self):
		self.get_logger().info("Running kalman filter ...") # remover
		self.get_logger().info("Aruco pose: {}".format(self.aruco_msg)) # remover apos testar
		self.kalman.kalman(self.imu_msg, self.aruco_msg)

		pose_est = self.kalman.x.A1
		
		pose = Pose()
		pose.position.x = pose_est[0]
		pose.position.y = pose_est[1]
		pose.position.z = 0.0
		q = tf_transformations.quaternion_from_euler(0, 0, pose_est[2])
		pose.orientation.x = q[0]
		pose.orientation.y = q[1]
		pose.orientation.z = q[2]
		pose.orientation.w = q[3]

		self.pub_.publish(pose)

	def isStopped(self, imu):
		return np.allclose(
			[self.imu_msg.linear_acceleration.x, self.imu_msg.linear_acceleration.y, self.imu_msg.angular_velocity.z],
			[imu.linear_acceleration.x, imu.linear_acceleration.y, imu.angular_velocity.z],
			0.1,
			0.1
		)

def main(args=None):
	rclpy.init(args=args)
	node = KalmanNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "main":
	main()
