from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='mobile_robot',
      executable='line_detector',
      name='line_detector'
    ),
    Node(
      package='mobile_robot',
      executable='maze_solver',
      name='maze_solver'
    )
  ])