import numpy as np

class Kalman:
	def __init__(self):
		self.x = np.matrix(np.zeros((3, 1)))
		self.P = np.matrix(np.eye(3))
		self.F = np.matrix(np.eye(3))
		self.u = np.matrix(np.zeros((3,1)))
		self.G = np.matrix(np.eye(3))
		self.V = np.matrix(np.eye(3))
		self.y = np.matrix(np.zeros((3,1)))
		self.H = np.matrix(np.eye(3))
		self.R = np.matrix(np.eye(3))

	def kalman(self, imu, aruco):
		# State prediction
		x_hat = self.F @ self.x + self.G @ self.u
		# Covariance prediction
		P_hat = self.F @ self.P @ self.F.T + self.V
		# Innovation
		v = self.y - self.H @ x_hat
		# Innovation covariance
		S = self.H @ P_hat @ self.H.T + self.R
		# Kalman gain
		K = P_hat @ self.H.T @ np.linalg.inv(S)
		# State update
		self.x = x_hat + K @ v
		# Covariance update
		self.P = (np.eye(3) - K @ self.H) @ P_hat

		print('Vel. Ang. X: ', imu.angular_velocity.x)
