from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    robot_ip = LaunchConfiguration('robot_ip')
    port = LaunchConfiguration('port')
    lidar_udp_port = LaunchConfiguration('lidar_udp_port')
    log_level = LaunchConfiguration('log_level')

    node = Node(
        package='roslinker',
        executable='roslinker_node',
        name='roslinker',
        namespace=robot_id,
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'robot_id': robot_id,
            'robot_ip': robot_ip,
            'port': port,
            'lidar_udp_port': lidar_udp_port,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='tb_01'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.101'),
        DeclareLaunchArgument('port', default_value='9000'),
        DeclareLaunchArgument('lidar_udp_port', default_value='5601'),
        DeclareLaunchArgument('log_level', default_value='info'),
        node,
    ])

