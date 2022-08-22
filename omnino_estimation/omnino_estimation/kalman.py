import numpy as np
import transforms3d

class Kalman:
	def __init__(self):
		self.x = np.matrix(np.zeros((3, 1)))  # State
		self.P = np.matrix(np.eye(3))  # State covariance matrix
		self.F = np.matrix(np.eye(3))  # State transition matrix
		self.u = np.matrix(np.zeros((3, 1)))  # Control vector
		self.G = np.matrix(np.eye(3))  # Control matrix
		self.y = np.matrix(np.zeros(3))  # Measurement vector
		self.H = np.matrix(np.eye(3))  # Observation matrix

		process_var = [0.002, 0.004, 0.008]
		# process_var = [0.0001, 0.0002, 0.0005] caso seja da posiçao e nao accel
		measurement_var = [1, 1, 1]

		# Estimated process error covariance
		self.V = np.matrix(np.diag(process_var))
		# Estimated measurement error covariance
		self.R = np.matrix(np.diag(measurement_var))

		self.imu_prev = None

	def getControlVector(self, imu_msg):
		return np.matrix(
			[
				[imu_msg.linear_acceleration.x],
				[imu_msg.linear_acceleration.y],
				[imu_msg.angular_velocity.z],
			]
		)

	def setControlMatrix(self, dt):
		self.G = np.matrix([[dt**2, 0, 0], [0, dt**2, 0], [0, 0, dt]])

	def getMeasurementVector(self, aruco_msg):
		# checar o tipo da mensagem e crir condiçao coisa util
		print("pose do aruco", aruco_msg)
		# tambem checar a funçao de quatertion para euler do transforms3d e substituir abaixo
		if len(aruco_msg) > 0:
			_, _, yaw = transforms3d.euler.quat2euler(aruco_msg.orientation, axes='xyzs')
			return np.matrix(
				[
					[aruco_msg.pose.position.x],
					[aruco_msg.pose.position.y],
					[np.degrees(yaw)],
				]
			)
		return np.matrix([[0], [0], [0]])

	def kalman(self, imu_msg, aruco_msg):
		if self.imu_prev == None:
			dt = 0.5
		else:
			dt = imu_msg.header.stamp - self.imu_prev.header.stamp

		# State prediction
		self.u = self.getControlVector(imu_msg)
		# self.get_logger().info('u: %s' % u)
		self.setControlMatrix(dt)
		x_hat = self.F @ self.x + self.G @ self.u

		# Covariance prediction
		P_hat = self.F @ self.P @ self.F.T + self.V

		# Innovation
		self.y = self.getMeasurementVector(aruco_msg)
		# self.get_logger().info('y: %s' % y)
		v = self.y - self.H @ x_hat

		# Innovation covariance
		S = self.H @ P_hat @ self.H.T + self.R

		# Kalman gain
		K = P_hat @ self.H.T @ np.linalg.inv(S)

		# State update
		self.x = x_hat + K @ v

		# Covariance update
		self.P = (np.eye(3) - K @ self.H) @ P_hat

		self.imu_prev = imu_msg
