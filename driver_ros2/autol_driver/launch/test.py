# -*- mode: Python -*-

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import launch

################### user configure parameters for ros2 start ###################


urdf_name = 'autol.urdf.xml'
urdf_file = os.path.join(get_package_share_directory('autol_driver')+'/urdf',urdf_name)
with open(urdf_file, 'r') as infp:
  robot_desc = infp.read()

autol_prefix = get_package_share_directory('autol_driver')
autol_config_dir = LaunchConfiguration('autol_config_dir', default=os.path.join(
                                                  autol_prefix, 'configuration_files'))
configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='autol.lua')

def generate_launch_description():
  robot_state_publisher_node = Node(
    package = 'robot_state_publisher',
    executable = 'robot_state_publisher',
    parameters=[
      {'robot_description': robot_desc},
      {'use_sim_time': False}],
    output = 'screen'
  )
  
  cartographer_node = Node(
    package = 'cartographer_ros',
    executable = 'cartographer_node',
    parameters = [{'use_sim_time': False}],
    arguments = [
      '-configuration_directory', autol_config_dir,
      '-configuration_basename', configuration_basename],
    output = 'screen'
  )
  
  return LaunchDescription([
    cartographer_node,
    robot_state_publisher_node
  ])
