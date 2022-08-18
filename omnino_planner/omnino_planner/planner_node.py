import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from ros2_aruco_interfaces.msg import ArucoMarkers

class PlannerNode(Node):
	def __init__(self):
		super().__init__('planner_node')
		self.pub_ = self.create_publisher(Pose, 'aruco_pose', 10)
		self.sub_ = self.create_subscription(ArucoMarkers, 'aruco_markers', self.aruco_callback, 10)
		# definir tf para base_link e guardar as poses

	def aruco_callback(self, aruco_msg):
		self.get_logger().info('Aruco msg: "%s"' % aruco_msg)
		# Fazer transformação e pegar pose atual do robo com base na primeira leitura do aruco
