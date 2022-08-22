import rclpy
from rclpy.node import Node

from omnino_estimation.kalman import Kalman
import tf_transformations

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose

class KalmanNode(Node):
	def __init__(self):
		super().__init__("kalman_node")
		self.kalman = Kalman()
		self.imu_msg = Imu()
		self.aruco_msg = Pose()
		self.imu_sub = self.create_subscription(Imu, "imu", self.imu_callback, 10)
		self.aruco_sub = self.create_subscription(Pose, "aruco_pose", self.aruco_callback, 10)
		self.pub_ = self.create_publisher(Pose, "imu_filtered", 10)

	def imu_callback(self, imu_msg):
		self.imu_msg = imu_msg
		self.kalman_callback()

	def aruco_callback(self, aruco_msg):
		self.aruco_msg = aruco_msg

	def kalman_callback(self):
		self.get_logger().info("Running kalman filter ...")
		self.kalman.kalman(self.imu_msg, self.aruco_msg)

		pose = Pose()
		pose.position.x = self.kalman.x[0][0]
		pose.position.y = self.kalman.x[0][1]
		pose.position.z = 0.0
		q = tf_transformations.quaternion_from_euler([0, 0, self.kalman.x[0][2]])
		pose.orientation.x = q[0]
		pose.orientation.x = q[1]
		pose.orientation.x = q[2]
		pose.orientation.x = q[3]

		self.pub_.publish(pose)

def main(args=None):
	rclpy.init(args=args)
	node = KalmanNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == "main":
	main()
