import rclpy
from  rclpy.node import Node
from omnino_estimation.kalman import Kalman

from sensor_msgs.msg import Imu
from ros2_aruco_interfaces.msg import ArucoMarkers

class KalmanSubscriber(Node):
	def __init__(self):
		super().__init__('kalman_filter')
		self.kalman = Kalman()
		self.imu_msg = Imu()
		self.aruco_msg = ArucoMarkers()
		self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
		self.aruco_sub = self.create_subscription(ArucoMarkers, 'aruco_markers', self.aruco_callback, 10)

	def imu_callback(self, imu_msg):
		self.imu_msg = imu_msg
		self.kalman_callback()

	def aruco_callback(self, aruco_msg):
		self.aruco_msg = aruco_msg

	def kalman_callback(self):
		self.get_logger().info('Running kalman filter ...')
		self.kalman.kalman(self.imu_msg, self.aruco_msg)
		self.get_logger().info('IMU: %f' % self.imu_msg.angular_velocity.x)
		if len(self.aruco_msg.poses) > 0:
			self.get_logger().info('ARUCO: %f' % self.aruco_msg.poses[0].position.z)

def main(args=None):
	rclpy.init(args=args)
	subs = KalmanSubscriber()
	rclpy.spin(subs)
	kalman.destroy_node()
	rclpy.shutdown()

if __name__ == 'main':
	main()
