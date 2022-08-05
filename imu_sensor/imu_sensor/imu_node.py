import rclpy
from rclpy.node import Node
from mpu6050 import mpu6050
import numpy as np

from sensor_msgs.msg import Imu

class ImuNode(Node):
	def __init__(self):
		super().__init__('imu_node')
		self.pub_ = self.create_publisher(Imu, 'imu', 10)
		self.imu_msg = Imu()
		self.imu_sensor = mpu6050(0x68)
		
		self.imu_msg.orientation_covariance = [
			0.002, 0.0, 0.0,
			0.0, 0.002, 0.0,
			0.0, 0.0, 0.002
		]
		
		self.imu_msg.angular_velocity_covariance = [
			0.02, 0.0, 0.0,
			0.0, 0.02, 0.0,
			0.0, 0.0, 0.02
		]
		
		self.imu_msg.linear_acceleration_covariance = [
			0.04, 0.0, 0.0,
			0.0, 0.04, 0.0,
			0.0, 0.0, 0.04
		]
		
		self.timer = self.create_timer(0.5, self.imu_callback)
		
	def imu_callback(self):
		accel_data = self.imu_sensor.get_accel_data()
		gyro_data = self.imu_sensor.get_gyro_data()
		
		# Quaternion
		#self.imu_msg.orientation.x =
		#self.imu_msg.orientation.y = 
		#self.imu_msg.orientation.z = 
		#self.imu_msg.orientation.w = 
		
		# Linear acceleration (m/s²)
		self.imu_msg.linear_acceleration.x = accel_data['x']
		self.imu_msg.linear_acceleration.y = accel_data['y']
		self.imu_msg.linear_acceleration.z = accel_data['z']

		# Angular velocity (rad/s)
		self.imu_msg.angular_velocity.x = gyro_data['x'] * np.pi/180
		self.imu_msg.angular_velocity.y = gyro_data['y'] * np.pi/180
		self.imu_msg.angular_velocity.z = gyro_data['z'] * np.pi/180
		
		self.pub_.publish(self.imu_msg)
		
def main(args=None):
	rclpy.init(args=args)
	
	imu_node = ImuNode()
	
	rclpy.spin(imu_node)
	
	imu_node.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
