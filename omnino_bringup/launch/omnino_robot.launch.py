from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='v4l2_camera',
			namespace='camera',
			output='screen',
			executable='v4l2_camera_node'
		),
		Node(
			package='imu_sensor',
			executable='imu_node'
		),
		Node(
			package='omnino_driver',
			executable='omnino_driver_node'
		),
		Node(
			package='omnino_estimation',
			executable='kalman_node'
		),
		Node(
			package='ros2_aruco',
			executable='aruco_node'
		),
	])
