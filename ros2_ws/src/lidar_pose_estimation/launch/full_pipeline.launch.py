from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # YDLidar driver
    ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')
    ydlidar_launch = os.path.join(ydlidar_share, 'launch', 'ydlidar_launch.py')
    
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch)
    )
    
    # Angle filter node
    filter_node = Node(
        package='lidar_pose_estimation',
        executable='filter_node',
        name='lidar_angle_filter',
        output='screen'
    )
    
    # Main pose estimation node
    pose_estimator = Node(
        package='lidar_pose_estimation',
        executable='pose_estimator',
        name='pose_estimator_node',
        output='screen',
        prefix='gnome-terminal --',
    )
    
    return LaunchDescription([
        driver,
        filter_node,
        pose_estimator,
    ])
