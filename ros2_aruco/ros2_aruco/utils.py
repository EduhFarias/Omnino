import numpy as np

def quaternion_from_euler(roll, pitch, yaw):
	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	return [qx, qy, qz, qw]

def euler_from_rvec(rvec):
	roll = np.arctan2(rvec[2, 1], rvec[2, 2])
	pitch = np.arctan2(-rvec[2, 0], np.sqrt(rvec[2, 1]**2 + rvec[2, 2]**2))
	yaw = np.arctan2(rvec[1, 0], rvec[0, 0])
	return [roll, pitch, yaw]
