from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    robot_ip = LaunchConfiguration('robot_ip')
    port = LaunchConfiguration('port')
    path = LaunchConfiguration('path')
    ws_url = LaunchConfiguration('ws_url')

    transport_mode = LaunchConfiguration('transport_mode')
    cmd_timeout_ms = LaunchConfiguration('cmd_timeout_ms')
    send_rate_hz = LaunchConfiguration('send_rate_hz')
    lidar_transport = LaunchConfiguration('lidar_transport')
    lidar_udp_port = LaunchConfiguration('lidar_udp_port')
    enable_lidar = LaunchConfiguration('enable_lidar')
    lidar_rate_hz = LaunchConfiguration('lidar_rate_hz')

    # optional: print/debug flags
    log_level = LaunchConfiguration('log_level')

    # Build ws url from robot_ip/port/path (original behavior)
    # e.g. ws://192.168.1.101:9000/ws
    ws_url_built = ['ws://', robot_ip, ':', port, path]

    # Use explicit ws_url if provided, otherwise use built one
    use_ws_url = IfCondition(PythonExpression(["'", ws_url, "' != ''"]))
    use_built = UnlessCondition(PythonExpression(["'", ws_url, "' != ''"]))

    # C++ bridge node
    bridge_node_ws_url = Node(
        package='tb_bridge_cpp',
        executable='tb_bridge_node',
        name='tb_bridge_node',
        namespace=robot_id,  # -> /tb_01, /tb_02, ...
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'robot_id': robot_id,
            'ws_url': ws_url,
            'robot_ip': robot_ip,
            'port': port,
            'path': path,
            'transport_mode': transport_mode,
            'cmd_timeout_ms': cmd_timeout_ms,
            'send_rate_hz': send_rate_hz,
            'lidar_transport': lidar_transport,
            'lidar_udp_port': lidar_udp_port,
            'enable_lidar': enable_lidar,
            'lidar_rate_hz': lidar_rate_hz,
        }],
        condition=use_ws_url
    )

    bridge_node_built = Node(
        package='tb_bridge_cpp',
        executable='tb_bridge_node',
        name='tb_bridge_node',
        namespace=robot_id,
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'robot_id': robot_id,
            'ws_url': ws_url_built,   # built url
            'robot_ip': robot_ip,
            'port': port,
            'path': path,
            'transport_mode': transport_mode,
            'cmd_timeout_ms': cmd_timeout_ms,
            'send_rate_hz': send_rate_hz,
            'lidar_transport': lidar_transport,
            'lidar_udp_port': lidar_udp_port,
            'enable_lidar': enable_lidar,
            'lidar_rate_hz': lidar_rate_hz,
        }],
        condition=use_built
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='tb_01'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.101'),
        DeclareLaunchArgument('port', default_value='9000'),
        DeclareLaunchArgument('path', default_value='/ws'),
        DeclareLaunchArgument('ws_url', default_value=''),  # if non-empty, overrides built URL

        DeclareLaunchArgument('transport_mode', default_value='ws_jsonl'),
        DeclareLaunchArgument('cmd_timeout_ms', default_value='500'),
        DeclareLaunchArgument('send_rate_hz', default_value='20.0'),

        DeclareLaunchArgument('lidar_transport', default_value='udp'),
        DeclareLaunchArgument('lidar_udp_port', default_value='5601'),
        DeclareLaunchArgument('enable_lidar', default_value='false'),
        DeclareLaunchArgument('lidar_rate_hz', default_value='1.0'),

        DeclareLaunchArgument('log_level', default_value='info'),

        bridge_node_ws_url,
        bridge_node_built,
    ])
