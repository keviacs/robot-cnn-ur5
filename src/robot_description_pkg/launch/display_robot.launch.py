import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
 
def generate_launch_description():
 
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
 
  robot_description_pkg_dir = get_package_share_directory('robot_description_pkg')
  urdf_file_name = 'my_first_robot.urdf'
  urdf = os.path.join(robot_description_pkg_dir, 'urdf', urdf_file_name)
 
  with open(urdf, 'r') as infp:
    robot_desc = infp.read()
 
  return LaunchDescription([
    DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
 
    Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}]
    ),
 
    Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='screen',
      arguments=['-d', os.path.join(robot_description_pkg_dir, 'rviz', 'config.rviz')]
    )
  ])