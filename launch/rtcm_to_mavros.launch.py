#!/usr/bin/env python3
"""Launch rtcm_to_mavros_node with configurable RTCM input parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create launch description for the RTCM-to-MAVROS publisher node."""
    rtcm_port_arg = DeclareLaunchArgument(
        "rtcm_port",
        default_value="",
        description="Local RTCM serial port (set this OR rtcm_tcp)",
    )
    rtcm_tcp_arg = DeclareLaunchArgument(
        "rtcm_tcp",
        default_value="",
        description="RTCM TCP endpoint in host:port format (set this OR rtcm_port)",
    )
    rtcm_baud_arg = DeclareLaunchArgument(
        "rtcm_baud",
        default_value="115200",
        description="RTCM serial baudrate (used by rtcm_port)",
    )
    output_topic_arg = DeclareLaunchArgument(
        "output_topic",
        default_value="/mavros/rtcm/send",
        description="Output topic for mavros_msgs/msg/RTCM",
    )
    reconnect_delay_arg = DeclareLaunchArgument(
        "reconnect_delay_s",
        default_value="2.0",
        description="Reconnect delay in seconds after input disconnect",
    )
    serial_timeout_arg = DeclareLaunchArgument(
        "serial_timeout_s",
        default_value="1.0",
        description="Input stream read timeout in seconds",
    )
    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="false",
        description="Enable verbose RTCM logging",
    )

    start_mavros_arg = DeclareLaunchArgument(
        "start_mavros",
        default_value="true",
        description="Also start mavros_node in this launch",
    )
    mavros_fcu_url_arg = DeclareLaunchArgument(
        "mavros_fcu_url",
        default_value="/dev/ttyTHS1:57600",
        description="MAVROS FCU URL (serial or udp)",
    )
    mavros_gcs_url_arg = DeclareLaunchArgument(
        "mavros_gcs_url",
        default_value="",
        description="MAVROS GCS URL",
    )
    mavros_tgt_system_arg = DeclareLaunchArgument(
        "mavros_tgt_system",
        default_value="1",
        description="MAVROS target system id",
    )
    mavros_tgt_component_arg = DeclareLaunchArgument(
        "mavros_tgt_component",
        default_value="1",
        description="MAVROS target component id",
    )
    mavros_pluginlists_yaml_arg = DeclareLaunchArgument(
        "mavros_pluginlists_yaml",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rtcm_serial_to_mavlink"), "config", "mavros_pluginlists.yaml"]
        ),
        description="Plugin allow/deny list YAML passed to mavros launch",
    )
    mavros_config_yaml_arg = DeclareLaunchArgument(
        "mavros_config_yaml",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rtcm_serial_to_mavlink"), "config", "mavros_config.yaml"]
        ),
        description="General MAVROS config YAML passed to mavros launch",
    )

    node = Node(
        package="rtcm_serial_to_mavlink",
        executable="rtcm_to_mavros_node",
        name="rtcm_to_mavros_node",
        output="screen",
        parameters=[
            {
                "rtcm_port": LaunchConfiguration("rtcm_port"),
                "rtcm_tcp": LaunchConfiguration("rtcm_tcp"),
                "rtcm_baud": LaunchConfiguration("rtcm_baud"),
                "output_topic": LaunchConfiguration("output_topic"),
                "reconnect_delay_s": LaunchConfiguration("reconnect_delay_s"),
                "serial_timeout_s": LaunchConfiguration("serial_timeout_s"),
                "debug": LaunchConfiguration("debug"),
            }
        ],
    )

    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "mavros.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("start_mavros")),
        launch_arguments={
            "fcu_url": LaunchConfiguration("mavros_fcu_url"),
            "gcs_url": LaunchConfiguration("mavros_gcs_url"),
            "tgt_system": LaunchConfiguration("mavros_tgt_system"),
            "tgt_component": LaunchConfiguration("mavros_tgt_component"),
            "pluginlists_yaml": LaunchConfiguration("mavros_pluginlists_yaml"),
            "config_yaml": LaunchConfiguration("mavros_config_yaml"),
        }.items(),
    )

    return LaunchDescription(
        [
            rtcm_port_arg,
            rtcm_tcp_arg,
            rtcm_baud_arg,
            output_topic_arg,
            reconnect_delay_arg,
            serial_timeout_arg,
            debug_arg,
            start_mavros_arg,
            mavros_fcu_url_arg,
            mavros_gcs_url_arg,
            mavros_tgt_system_arg,
            mavros_tgt_component_arg,
            mavros_pluginlists_yaml_arg,
            mavros_config_yaml_arg,
            mavros_launch,
            node,
        ]
    )
