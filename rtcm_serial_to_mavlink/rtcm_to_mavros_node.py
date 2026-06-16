#!/usr/bin/env python3
"""ROS 2 node that publishes RTCM frames to mavros_msgs/RTCM.

This node consumes a mixed RTCM stream from either:
- a local serial port, or
- a TCP endpoint (host:port)

It uses pyrtcm.RTCMReader for RTCM v3 detection/validation and publishes the
validated raw RTCM bytes as mavros_msgs/msg/RTCM.
"""

from __future__ import annotations

import contextlib
import threading
from typing import Optional

import rclpy
from mavros_msgs.msg import RTCM
from pyrtcm import RTCMReader
from rclpy.node import Node
import serial


class RTCMToMavrosNode(Node):
    """Bridge RTCM stream input to mavros_msgs/RTCM output."""

    def __init__(self) -> None:
        super().__init__("rtcm_to_mavros_node")

        self.declare_parameter("rtcm_port", "")
        self.declare_parameter("rtcm_tcp", "")
        self.declare_parameter("rtcm_baud", 115200)
        self.declare_parameter("output_topic", "/mavros/rtcm/send")
        self.declare_parameter("reconnect_delay_s", 2.0)
        self.declare_parameter("serial_timeout_s", 1.0)
        self.declare_parameter("debug", False)

        self._rtcm_port = self.get_parameter("rtcm_port").get_parameter_value().string_value.strip()
        self._rtcm_tcp = self.get_parameter("rtcm_tcp").get_parameter_value().string_value.strip()
        self._rtcm_baud = int(self.get_parameter("rtcm_baud").get_parameter_value().integer_value)
        self._output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._reconnect_delay_s = (
            self.get_parameter("reconnect_delay_s").get_parameter_value().double_value
        )
        self._serial_timeout_s = self.get_parameter("serial_timeout_s").get_parameter_value().double_value
        self._debug = self.get_parameter("debug").get_parameter_value().bool_value

        if bool(self._rtcm_port) == bool(self._rtcm_tcp):
            raise ValueError("Set exactly one of 'rtcm_port' or 'rtcm_tcp'.")

        self._publisher = self.create_publisher(RTCM, self._output_topic, 10)

        self._serial: Optional[serial.SerialBase] = None
        self._reader: Optional[RTCMReader] = None

        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(target=self._read_loop, name="rtcm-reader", daemon=True)
        self._reader_thread.start()

        self.get_logger().info(
            "Started. input=%s output_topic=%s",
            f"tcp:{self._rtcm_tcp}" if self._rtcm_tcp else f"serial:{self._rtcm_port}@{self._rtcm_baud}",
            self._output_topic,
        )

    def _connect_input(self) -> None:
        """Connect input stream and initialize RTCMReader."""
        while not self._stop_event.is_set():
            try:
                if self._rtcm_tcp:
                    self.get_logger().info(f"[RTCM] connecting tcp={self._rtcm_tcp}")
                    self._serial = serial.serial_for_url(
                        f"socket://{self._rtcm_tcp}",
                        timeout=self._serial_timeout_s,
                        write_timeout=self._serial_timeout_s,
                    )
                    self.get_logger().info(f"[RTCM] connected tcp={self._rtcm_tcp}")
                else:
                    self.get_logger().info(
                        f"[RTCM] connecting serial={self._rtcm_port} baud={self._rtcm_baud}"
                    )
                    self._serial = serial.Serial(
                        port=self._rtcm_port,
                        baudrate=self._rtcm_baud,
                        timeout=self._serial_timeout_s,
                        write_timeout=self._serial_timeout_s,
                    )
                    self.get_logger().info(f"[RTCM] connected serial={self._rtcm_port}")

                self._reader = RTCMReader(self._serial)
                return
            except (serial.SerialException, OSError, ValueError, RuntimeError) as err:
                self.get_logger().warning(
                    f"[RTCM] connect failed: {err}; retrying in {self._reconnect_delay_s:.1f}s"
                )
                self._stop_event.wait(self._reconnect_delay_s)

    def _reset_input(self) -> None:
        """Close current input resources."""
        if self._serial is not None:
            with contextlib.suppress(serial.SerialException, OSError):
                self._serial.close()
        self._serial = None
        self._reader = None

    def _publish_rtcm(self, raw_frame: bytes) -> None:
        """Publish one validated RTCM frame as mavros_msgs/RTCM."""
        msg = RTCM()
        msg.data = list(raw_frame)
        self._publisher.publish(msg)

    def _read_loop(self) -> None:
        """Continuously parse RTCM and publish validated frames."""
        while not self._stop_event.is_set():
            if self._reader is None:
                self._connect_input()
                if self._reader is None:
                    continue

            try:
                raw, parsed = self._reader.read()
                if not raw:
                    continue

                self._publish_rtcm(bytes(raw))

                if self._debug:
                    identity = getattr(parsed, "identity", "unknown")
                    self.get_logger().info(f"[RTCM] type={identity} len={len(raw)} published")
            except (serial.SerialException, OSError, EOFError) as err:
                self.get_logger().warning(f"[RTCM] stream disconnected: {err}")
                self._reset_input()
            except (ValueError, TypeError, RuntimeError) as err:
                if self._debug:
                    self.get_logger().warning(f"[RTCM] parser recovered from malformed bytes: {err}")

    def destroy_node(self) -> bool:
        """Stop reader thread and release resources before node shutdown."""
        self._stop_event.set()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        self._reset_input()
        return super().destroy_node()


def main() -> None:
    """ROS 2 entrypoint."""
    rclpy.init()

    node: Optional[RTCMToMavrosNode] = None
    try:
        node = RTCMToMavrosNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
