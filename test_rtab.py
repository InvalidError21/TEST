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

urdf_name = 'test.urdf.xml'
urdf_file = os.path.join(get_package_share_directory('autol_driver') + '/urdf', urdf_name)
with open(urdf_file, 'r') as infp:
  robot_desc = infp.read()

autol_prefix = get_package_share_directory('autol_driver')
autol_config_dir = LaunchConfiguration('autol_config_dir', default=os.path.join(
                                                  autol_prefix, 'configuration_files'))
configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='test.lua')

def generate_launch_description():
             
  autol_driver = Node(
    package='autol_driver',
    executable='autol_driver_node',
    name='autol_lidar_publisher',
    output='screen',
    parameters=autol_node_parameters,
    remappings=[
      ('/autol_pointcloud_1', '/scan_cloud')
    ]
  )

  rtabmap_odom = Node(
    package='rtabmap_odom',
    executable='icp_odometry',
    output='screen',
    parameters=[{
      'frame_id': 'base_link',
      'odom_frame_id': 'odom',
      'publish_tf': True,
      'wait_for_transform': 0.2,
      'expected_update_rate': 15.0,
      'publish_null_when_lost': False,
    }],
    arguments=[
      'Icp/PointToPlane', 'false',
      'Icp/Iterations', '10',
      'Icp/VoxelSize', '0.2',
      'Icp/Epsilon', '0.001',
      'Icp/PointToPlaneK', '20',
      'Icp/PointToPlaneRadius', '0',
      'Icp/MaxTranslation', '2',
      'Icp/MaxCorrespondenceDistance', '1',
      'Icp/Strategy', '1',
      'Icp/OutlierRatio', '0.7',
      'Icp/CorrespondenceRatio', '0.01',
      'Odom/ScanKeyFrameThr', '0.4',
      'OdomF2M/ScanSubtractRadius', '0.1',
      'OdomF2M/ScanMaxSize', '15000',
      'OdomF2M/BundleAdjustment', 'false',
      'Odom/GuessMotion', 'true',
      'Odom/ResetCountdown', '1',
    ]
  )
  
  rtabmap_slam = Node(
    package='rtabmap_slam',
    executable='rtabmap',
    output='screen',
    parameters=[{
      'frame_id': 'base_link',
      'odom_frame_id': 'odom',
      'map_frame_id': 'map',
      'publish_tf': True,
      'subscribe_depth': False,
      'subscribe_rgb': False,
      'subscribe_scan': False,
      'subscribe_scan_cloud': True,
      'approx_sync': True,
      'wait_for_transform': 0.2,
      'map_always_update': True,
      }],
    arguments=[
      'Mem/IncrementalMemory', 'true',  # Ensuring incremental memory for mapping
      'RGBD/CreateOccupancyGrid', 'true',
      'RGBD/ProximityMaxGraphDepth', '0',
      'RGBD/ProximityPathMaxNeighbors', '1',
      'RGBD/AngularUpdate', '0.05',
      'RGBD/LinearUpdate', '0.05',
      'Mem/NotLinkedNodesKept', 'false',
      'Mem/STMSize', '30',
      'Mem/LaserScanNormalK', '20',
      'Reg/Strategy', '1',
      'Icp/VoxelSize', '0.1',
      'Icp/PointToPlaneK', '20',
      'Icp/PointToPlaneRadius', '0',
      'Icp/PointToPlane', 'true',
      'Icp/Iterations', '10',
      'Icp/Epsilon', '0.001',
      'Icp/MaxTranslation', '3',
      'Icp/MaxCorrespondenceDistance', '1',
      'Icp/Strategy', '1',
      'Icp/OutlierRatio', '0.7',
      'Icp/CorrespondenceRatio', '0.2',
      'Grid/Sensor', '0',
      'Kp/MaxFeatures', '-1',
    ]
  )
    
  rtabmap_viz = Node(
    package='rtabmap_viz',
    executable='rtabmap_viz',
    output='screen',
    parameters=[{
      'subscribe_scan_cloud': True,
      'frame_id': 'base_link',
      'odom_frame_id': 'odom',
      'approx_sync': True,
    }],
  )
  
  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_desc}],
    output='screen'
  )

  return LaunchDescription([
    autol_driver,
    rtabmap_odom,
    rtabmap_slam,
    rtabmap_viz,
    robot_state_publisher
  ])
