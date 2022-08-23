import rclpy
from rclpy.node import Node

import numpy as np
from tf2_ros import TransformBroadcaster
import tf_transformations

from geometry_msgs.msg import Pose, TransformStamped
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

		self.pub_ = self.create_publisher(Pose, "aruco_pose", 10)
		self.sub_aruco_ = self.create_subscription(ArucoMarkers, "aruco_markers", self.aruco_callback, 10)
		self.sub_imu_ = self.create_subscription(Pose, "imu_filtered", self.imu_callback, 10)

		self.br = TransformBroadcaster(self)

	def aruco_callback(self, aruco_msg):
		if self.initialPose.position == -1.0:
			self.get_logger().info("Setting initial pose")

			self.initialPose.position.x = aruco_msg.poses[0].position.x
			self.initialPose.position.y = aruco_msg.poses[0].position.y
			self.initialPose.position.z = 0.0
			self.initialPose.rotation.x = aruco_msg.poses[0].orientation.x
			self.initialPose.rotation.y = aruco_msg.poses[0].orientation.y
			self.initialPose.rotation.z = aruco_msg.poses[0].orientation.z
			self.initialPose.rotation.w = aruco_msg.poses[0].orientation.w

			self.previusPose = np.copy(self.initialPose)
		else:
			t = TransformStamped()
			t.header.stamp = self.get_clock().now().to_msg()
			t.header.frame_id = "world"
			t.child_frame_id = "base_link"

			t.transform.translation.x = aruco_msg.poses[0].position.x - self.initialPose.position.x
			t.transform.translation.y = aruco_msg.poses[0].position.y - self.initialPose.position.y
			t.transform.translation.z = 0.0

			q = quaternion_diff(self.initialPose, aruco_msg.poses[0])

			t.transform.rotation.x = q[0]
			t.transform.rotation.y = q[1]
			t.transform.rotation.z = q[2]
			t.transform.rotation.w = q[3]

			self.br.sendTransform(t)

			self.pub_callback(aruco_msg.poses[0])

	def imu_callback(self, imu_msg):
		t = TransformStamped()
		t.header.stamp = self.get_clock().now().to_msg()
		t.header.frame_id = "world"
		t.child_frame_id = "imu_filtered"

		t.transform.translation.x = imu_msg.position.x
		t.transform.translation.y = imu_msg.position.y
		t.transform.translation.z = imu_msg.position.z
		t.transform.rotation.x = imu_msg.orientation.x
		t.transform.rotation.y = imu_msg.orientation.y
		t.transform.rotation.z = imu_msg.orientation.z
		t.transform.rotation.w = imu_msg.orientation.w

		self.br.sendTransform(t)

	def pub_callback(self, aruco_msg):
		pose = Pose()

		pose.position.x = aruco_msg.position.x - self.previusPose.position.x
		pose.position.x = aruco_msg.position.y - self.previusPose.position.y
		pose.position.z = 0.0

		q = quaternion_diff(self.previusPose, aruco_msg)

		pose.orientation.x = q[0]
		pose.orientation.y = q[1]
		pose.orientation.z = q[2]
		pose.orientation.w = q[3]

		self.pub_.publish(pose)

def main(args=None):
	rclpy.init(args=args)
	node = PlannerNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "main":
	main()
