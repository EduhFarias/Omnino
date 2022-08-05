from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='v4l2_camera',
			namespace='camera',
			output='screen',
			executable='v4l2_camera_node',
			name='cam'
		)
	])
