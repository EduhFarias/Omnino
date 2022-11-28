import rclpy
from rclpy.node import Node

import numpy as np
from tf2_ros import TransformBroadcaster
import tf_transformations
import json

from geometry_msgs.msg import Pose, TransformStamped, Twist
from sensor_msgs.msg import Imu
from ros2_aruco_interfaces.msg import ArucoMarkers

class PlannerNode(Node):
	def __init__(self):
		super().__init__("planner_node")

		self.initialPose = Pose()
		self.initialPose.position.z = -1.0
		self.previusPose = Pose()

		self.ax = []
		self.ay = []
		self.vz = []
		self.saved = False

		self.i = 1
		self.dt = 0.5

		self.declare_parameter('path', 'linear')

		path_param = self.get_parameter('path').get_parameter_value().string_value

		if path_param == 'linear':
			# Linear path
			self.path = [
					np.linspace(0.0, 0.0, 10),
					np.linspace(1.0, 1.0, 10),
					np.linspace(0.0, 0.0, 10)
				]
			
		elif path_param == 'L':
			# L-shaped path
			self.path = [
					np.append(np.linspace(0.0, 0.0, 10), np.linspace(1.0, 1.0, 10)),
					np.append(np.linspace(1.0, 1.0, 10), np.linspace(0.0, 0.0, 10)),
					np.linspace(0.0, 0.0, 20)
				]

		elif path_param == 'curve':
			# S-shaped path
			alpha = 1
			t = np.linspace(0, np.pi, 30)
			x = alpha * (-1) * np.sqrt(2) * (np.sin(t) * (np.sin(t)**2 + 2 * np.cos(t)**2 + 1)) / (np.sin(t)**2 + 1)**2
			y = alpha * (-1) * np.sqrt(2) * (np.sin(t)**2 + np.sin(t)**4 + np.cos(t)**2 * (-1 + np.sin(t)**2)) / (np.sin(t)**2 + 1)**2
			w = np.arccos(np.clip(np.dot(x, y)/np.linalg.norm(x)/np.linalg.norm(y), -1, 1))
			self.path = [x, y, np.linspace(0, 0, 30)]

		else:
			# Lemniscate path
			alpha = 1
			t = np.linspace(0, 2*np.pi, 30)
			x = alpha * (-1) * np.sqrt(2) * (np.sin(t) * (np.sin(t)**2 + 2 * np.cos(t)**2 + 1)) / (np.sin(t)**2 + 1)**2
			y = alpha * (-1) * np.sqrt(2) * (np.sin(t)**2 + np.sin(t)**4 + np.cos(t)**2 * (-1 + np.sin(t)**2)) / (np.sin(t)**2 + 1)**2
			w = np.arccos(np.clip(np.dot(x, y)/np.linalg.norm(x)/np.linalg.norm(y), -1, 1))
			self.path = [x, y, np.linspace(0, 0, 30)]

		self.origin = Pose()
		self.origin.position.x = 0.6 	# Em metros
		self.origin.position.y = 0.95 	# Em metros
		self.origin.position.z = 1.5 	# Em metros
		self.origin.orientation.x = 0.0
		self.origin.orientation.y = 0.0
		self.origin.orientation.z = 1.0
		self.origin.orientation.w = 0.0

		self.pub_cmd_vel_ = self.create_publisher(Twist, 'cmd_vel', 10)
		self.pub_aruco_pose_ = self.create_publisher(Pose, 'aruco_pose', 10)

		self.sub_aruco_ = self.create_subscription(ArucoMarkers, "aruco_markers", self.aruco_callback, 10)
		self.sub_pose_ = self.create_subscription(Pose, "estimated_pose", self.pose_callback, 10)
		self.timer = self.create_timer(0.5, self.cmd_vel_callback)
		self.sub_imu_ = self.create_subscription(Imu, "imu", self.imu_callback, 10)

		self.br = TransformBroadcaster(self)

	def aruco_callback(self, aruco_msg):
		# aruco_msg é composto de header, ids, poses.
		# tratar qual o id ou ids serão utilizados aqui, caso so um mesmo: poses[0] -> .position e .orientation
		
		# Vector-Quaternion: World-ArUco
		t_wa = np.array([self.origin.position.x, self.origin.position.y, self.origin.position.z])
		q_wa = np.array([self.origin.orientation.x, self.origin.orientation.y, self.origin.orientation.z, self.origin.orientation.w])

		# Vector-Quaternion: Robot-ArUco
		t_ra = np.array([aruco_msg.poses[0].position.x, aruco_msg.poses[0].position.y, aruco_msg.poses[0].position.z])
		q_ra = np.array([aruco_msg.poses[0].orientation.x, aruco_msg.poses[0].orientation.y, aruco_msg.poses[0].orientation.z, aruco_msg.poses[0].orientation.w])

		# Vector-Quaternion: World-Robot
		q_wr = tf_transformations.quaternion_multiply(q_wa, tf_transformations.quaternion_inverse(q_ra))
		t_wr = t_wa - self.qv_mult(q_wr, t_ra)

		aruco_pose = Pose()
		aruco_pose.position.x = t_wr[0]
		aruco_pose.position.y = t_wr[1]
		aruco_pose.position.z = t_wr[2]
		aruco_pose.orientation.x = q_wr[0]
		aruco_pose.orientation.y = q_wr[1]
		aruco_pose.orientation.z = q_wr[2]
		aruco_pose.orientation.w = q_wr[3]

		self.pub_aruco_pose_.publish(aruco_pose)

	def qv_mult(self, q1, v1):
		# comment this out if v1 doesn't need to be a unit vector
		# v1 = tf_transformations.unit_vector(v1)
		q2 = list(v1)
		q2.append(0.0)
		return tf_transformations.quaternion_multiply(
			tf_transformations.quaternion_multiply(q1, q2),
			tf_transformations.quaternion_conjugate(q1)
		)[:3]

	def pose_callback(self, pose_msg):
		self.sendTf(
			'world',
			'estimated_pose',
			pose_msg.position.x,
			pose_msg.position.y,
			pose_msg.position.z,
			pose_msg.orientation.x,
			pose_msg.orientation.y,
			pose_msg.orientation.z,
			pose_msg.orientation.w
		)

	def cmd_vel_callback(self):
		vel = Twist()
		vel.linear.x = 0.0
		vel.linear.y = 0.0
		vel.linear.z = 0.0
		vel.angular.x = 0.0
		vel.angular.y = 0.0
		vel.angular.z = 0.0

		if self.i == len(self.path[0])-1:
			self.pub_cmd_vel_.publish(vel)
			return
		
		vel.linear.x = self.path[0][self.i]
		vel.linear.y = self.path[1][self.i]
		vel.angular.z = self.path[2][self.i]
		self.pub_cmd_vel_.publish(vel)
		self.i += 1

	def imu_callback(self, imu_msg):
		if self.saved:
			return
		if self.i == len(self.path[0]) - 1:
			with open('imu-' + self.get_parameter('path').get_parameter_value().string_value + '.json', 'w') as outp:
				json.dump({'x': self.ax, 'y': self.ay, 'z': self.vz}, outp)
			self.saved = True
		else:
			self.ax.append(imu_msg.linear_acceleration.x)
			self.ay.append(imu_msg.linear_acceleration.y)
			self.vz.append(imu_msg.angular_velocity.z)

	def sendTf(self, header_id, child_id, x, y, z, q0, q1, q2, q3):
		t = TransformStamped()
		t.header.stamp = self.get_clock().now().to_msg()
		t.header.frame_id = header_id
		t.child_frame_id = child_id
		t.transform.translation.x = x
		t.transform.translation.y = y
		t.transform.translation.z = z
		t.transform.rotation.x = q0
		t.transform.rotation.y = q1
		t.transform.rotation.z = q2
		t.transform.rotation.w = q3
		self.br.sendTransform(t)

def main(args=None):
	rclpy.init(args=args)
	node = PlannerNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "main":
	main()
