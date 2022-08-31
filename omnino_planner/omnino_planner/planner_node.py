import rclpy
from rclpy.node import Node

import numpy as np
from tf2_ros import TransformBroadcaster
import tf_transformations

from geometry_msgs.msg import Pose, TransformStamped, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers

def quaternion_diff(pose1, pose2):
	return tf_transformations.quaternion_multiply(
		[
			pose2.orientation.x,
			pose2.orientation.y,
			pose2.orientation.z,
			pose2.orientation.w
		],
		tf_transformations.quaternion_inverse(
			[
				pose1.orientation.x,
				pose1.orientation.y,
				pose1.orientation.z,
				pose1.orientation.w,
			]
		)
	)

class PlannerNode(Node):
	def __init__(self):
		super().__init__("planner_node")

		self.initialPose = Pose()
		self.initialPose.position.z = -1.0
		self.previusPose = Pose()

		self.i = 1
		self.dt = 0.5

		self.declare_parameter('path', 'linear')

		path_param = self.get_paramater('path').get_parameter_value().string_value

		if path_param == 'curve':
			self.path = []

		elif path_param == 'lemniscate':
			alpha = 1
			t = np.linspace(0, 2*np.pi, 100)
			self.path = [
				alpha * np.sqrt(2) * np.cos(t) / (np.sin(t)**2 + 1),
				alpha * np.sqrt(2) * np.cos(t) * np.sin(t) / (np.sin(t)**2 + 1),
				np.linspace(0.0, 0.0, 100)
			]

		elif path_param == 'L':
			self.path = [
				np.append(np.linspace(0.0, 0.0, 50), np.linspace(0.0, 1.0, 50)),
				np.append(np.linspace(0.0, 3.0, 50), np.linspace(3.0, 3.0, 50)),
				np.linspace(0.0, 0.0, 100)
			]

		else:
			# Linear path
			self.path = [
				np.linspace(0.0, 0.0, 50),
				np.linspace(0.0, 1.0, 50),
				np.linspace(0.0, 0.0, 50)
			]

		self.pub_pose_ = self.create_publisher(Pose, "aruco_pose", 10)
		self.pub_cmd_vel_ = self.create_publisher(Twist, 'cmd_vel', 10)

		self.sub_aruco_ = self.create_subscription(ArucoMarkers, "aruco_markers", self.aruco_callback, 10)
		self.sub_imu_ = self.create_subscription(Pose, "imu_filtered", self.imu_callback, 10)

		self.br = TransformBroadcaster(self)

	def aruco_callback(self, aruco_msg):
		if self.initialPose.position.z == -1.0:
			self.get_logger().info("Setting initial pose")

			self.initialPose.position.x = aruco_msg.poses[0].position.x
			self.initialPose.position.y = aruco_msg.poses[0].position.y
			self.initialPose.position.z = 0.0
			self.initialPose.rotation.x = aruco_msg.poses[0].orientation.x
			self.initialPose.rotation.y = aruco_msg.poses[0].orientation.y
			self.initialPose.rotation.z = aruco_msg.poses[0].orientation.z
			self.initialPose.rotation.w = aruco_msg.poses[0].orientation.w

			self.previusPose = np.copy(self.initialPose)
			self.sendTf('world', 'base_link', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
		else:
			q = quaternion_diff(self.initialPose, aruco_msg.poses[0])
			self.sendTf(
				'world',
				'base_link',
				aruco_msg.poses[0].position.x - self.initialPose.position.x,
				aruco_msg.poses[0].position.y - self.initialPose.position.y,
				0.0,
				q[0], q[1], q[2], q[3]
			)

			pose = Pose()
			pose.position.x = aruco_msg.poses[0].position.x - self.previusPose.position.x
			pose.position.x = aruco_msg.poses[0].position.y - self.previusPose.position.y
			pose.position.z = 0.0
			q = quaternion_diff(self.previusPose, aruco_msg.poses[0])
			pose.orientation.x = q[0]
			pose.orientation.y = q[1]
			pose.orientation.z = q[2]
			pose.orientation.w = q[3]
			self.pub_pose_.publish(pose)

	def imu_callback(self, imu_msg):
		self.sendTf(
			'world',
			'imu_filtered',
			imu_msg.position.x,
			imu_msg.position.y,
			imu_msg.position.z,
			imu_msg.orientation.x,
			imu_msg.orientation.y,
			imu_msg.orientation.z,
			imu_msg.orientation.w
		)

		vel = Twist()
		vel.linear.x = 0.0
		vel.linear.y = 0.0
		vel.linear.z = 0.0
		vel.angular.x = 0.0
		vel.angular.y = 0.0
		vel.angular.z = 0.0

		if self.isClose(imu_msg) or self.i == len(self.path[0])-1:
			self.pub_cmd_vel_.publish(vel)
			return
		
		vel.linear.x = (self.path[0][self.i] - imu_msg.position.x) / self.dt
		vel.linear.y = (self.path[1][self.i] - imu_msg.position.y) / self.dt
		_, _, z = tf_transformations.euler_from_quaternion([imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])
		vel.angular.z = (self.path[2][self.i] - z) / self.dt
		self.i += 1
		self.pub_cmd_vel_.publish(vel)

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

	def isClose(self, imu):
		return np.allclose(
			self.path,
			[imu.position.x, imu.position.y, imu.orientation.z],
			0.1,
			0.1,
		)

def main(args=None):
	rclpy.init(args=args)
	node = PlannerNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "main":
	main()
