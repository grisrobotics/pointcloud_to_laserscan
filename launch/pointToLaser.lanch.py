from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='PointToLaser',
            executable='PointToLaser_node',
            name='point_to_laser',
            parameters= [
                {'param_topic_in', 'livox/lidar'},
                {'param_topic_out', 'scan'}]
        )
    ])