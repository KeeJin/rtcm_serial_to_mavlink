#!/usr/bin/env python3
"""Launch rtcm_to_mavros_node with configurable RTCM input parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    return LaunchDescription(
        [
            rtcm_port_arg,
            rtcm_tcp_arg,
            rtcm_baud_arg,
            output_topic_arg,
            reconnect_delay_arg,
            serial_timeout_arg,
            debug_arg,
            node,
        ]
    )
