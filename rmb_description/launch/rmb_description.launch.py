from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource 

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('rmb_description'),
                             'urdf', 'rmb.xacro.urdf')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )
    
    rplidar_package_share_path = get_package_share_path('rplidar_ros') 

    rplidar_launch_file_path = os.path.join(rplidar_package_share_path, 'launch', 'rplidar_a2m8_launch.py')

    
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rplidar_launch_file_path])
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rplidar_launch, 
    ])