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
manufacture_id = 'autol'
model_id = 'G32'
input_type = 1
frame_rate = 25

lidar_count = 1
lidar_port_1 = 5001
lidar_port_2 = 5002
lidar_port_3 = 5003
lidar_port_4 = 5004
lidar_port_5 = 5005
lidar_port_6 = 5006

pcap_path = ''
packet_per_frame = 180
read_once = 0
read_fast = 0
repeat_delay = 0.0
calibration = False

rviz_config=get_package_share_directory('autol_driver')+'/rviz/pointcloud2_config.rviz'

autol_node_parameters = [
  #Sensor Parameter
    {"manufacture_id": manufacture_id},
    {"model_id": model_id},
    {"input_type" : input_type},
    {"frame_rate": frame_rate},
  #Socket Parameter
    {"lidar_count": lidar_count},
    {"lidar_port_1": lidar_port_1},
    {"lidar_port_2": lidar_port_2},
    {"lidar_port_3": lidar_port_3},
    {"lidar_port_4": lidar_port_4},
    {"lidar_port_5": lidar_port_5},
    {"lidar_port_6": lidar_port_6},
  #Pcap Parameter
    {"pcap_path": pcap_path},
    {"packet_per_frame": packet_per_frame},
    {"read_once": read_once},
    {"read_fast": read_fast},
    {"repeat_delay": repeat_delay},
  #calibration
    {"calibration" : calibration},
]

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



  autol_driver = Node(
    package='autol_driver',
    executable='autol_driver_node',
    name='autol_lidar_publisher',
    output='screen',
    parameters=autol_node_parameters,
    remappings=[
    	('/autol_pointcloud_1', '/points2')
    ]
  )

  autol_rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz',
    output='screen',
    arguments=['-d',rviz_config]
  )
  
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
  
  cartographer_occupancy_grid_node = Node(
    package = 'cartographer_ros',
    executable = 'cartographer_occupancy_grid_node',
    parameters = [
      {'use_sim_time': False},
      {'resolution': 0.05}],
  )

  return LaunchDescription([
    autol_driver,
    autol_rviz,
    cartographer_node,
    cartographer_occupancy_grid_node,
    robot_state_publisher_node
  ])
