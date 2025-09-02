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

    # ==== Nieuwe Static TF Publisher Node voor de lidar ====
    # Deze node publiceert de transformatie van 'base_link' naar 'laser'.
    # De 'laser' frame is het frame waarin de RPLIDAR zijn scans publiceert.
    # De waarden '0 0 0 0 0 0' zijn de vertaling (x,y,z) en rotatie (roll,pitch,yaw)
    # ten opzichte van het 'parent_frame_id' (hier 'base_link').
    # Je moet deze waarden aanpassen aan de fysieke plaatsing van je lidar op je robot.
    # Bijvoorbeeld:
    # '0.1' '0' '0.2' '0' '0' '0' zou betekenen 10cm naar voren, 20cm omhoog, geen rotatie.
    static_tf_publisher_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_rplidar', # Geef het een unieke naam
        output='screen',
        arguments=[
            '0.0', '0.0', '0.1',  # x, y, z translatie (aanpassen!)
            '0.0', '0.0', '-0.78539816339745',  # roll, pitch, yaw rotatie (aanpassen!)
            'base_link',         # parent_frame_id (dit is meestal het basisframe van je robot)
            'laser'              # child_frame_id (dit is het frame van de RPLIDAR-scan)
        ]
    )
    # =======================================================
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rplidar_launch,
        static_tf_publisher_lidar_node, 
    ])