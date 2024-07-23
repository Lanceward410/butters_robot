import os
from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
 
  return LaunchDescription([
    Node(package='tcp_communications_pkg', executable='client_node',
      output='screen'),
  ])
