#!/usr/bin/env python3
"""Bridge RTCM v3 frames from a mixed serial stream into MAVLink GPS_RTCM_DATA.

This program reads a serial byte stream that may contain arbitrary binary data,
extracts only valid RTCM v3 frames using pyrtcm.RTCMReader, and forwards those
raw RTCM bytes to ArduPilot over MAVLink 2 as GPS_RTCM_DATA (message 233).
"""

from __future__ import annotations

import argparse
import contextlib
import logging
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import Generator, Optional, Tuple

# Force MAVLink 2 framing in pymavlink.
os.environ.setdefault("MAVLINK20", "1")

import serial
from pymavlink import mavutil
from pyrtcm import RTCMReader

LOGGER = logging.getLogger("rtcm_bridge")

MAVLINK_RTCM_MAX_DATA_LEN = 180
MAVLINK_RTCM_MAX_FRAGMENTS = 4
MAVLINK_RTCM_MAX_FRAME_LEN = MAVLINK_RTCM_MAX_DATA_LEN * MAVLINK_RTCM_MAX_FRAGMENTS


@dataclass(frozen=True)
class BridgeConfig:
    """Runtime configuration for the RTCM bridge."""

    rtcm_port: str
    rtcm_baud: int
    mavlink_udp: Optional[str]
    mavlink_serial: Optional[str]
    mavlink_baud: int
    source_system: int
    source_component: int
    debug: bool
    monitor_gps: bool = False
    gps_log_interval_s: float = 1.0
    reconnect_delay_s: float = 2.0
    serial_timeout_s: float = 1.0


class RTCMSerialReader:
    """Read and validate RTCM frames from a mixed serial stream.

    This class owns both the pyserial connection and the pyrtcm RTCMReader.
    RTCMReader handles stream synchronization and CRC validation internally.
    """

    def __init__(
        self,
        port: str,
        baudrate: int,
        reconnect_delay_s: float = 2.0,
        timeout_s: float = 1.0,
    ) -> None:
        self._port = port
        self._baudrate = baudrate
        self._reconnect_delay_s = reconnect_delay_s
        self._timeout_s = timeout_s

        self._serial: Optional[serial.Serial] = None
        self._reader: Optional[RTCMReader] = None

    def _connect(self) -> None:
        """Open serial port and initialize RTCMReader, retrying until success."""
        while True:
            try:
                LOGGER.info("[RTCM] connecting serial=%s baud=%d", self._port, self._baudrate)
                self._serial = serial.Serial(
                    port=self._port,
                    baudrate=self._baudrate,
                    timeout=self._timeout_s,
                    write_timeout=self._timeout_s,
                )
                self._reader = RTCMReader(self._serial)
                LOGGER.info("[RTCM] connected serial=%s", self._port)
                return
            except (serial.SerialException, OSError, ValueError, RuntimeError) as err:
                LOGGER.warning("[RTCM] connect failed: %s; retrying in %.1fs", err, self._reconnect_delay_s)
                time.sleep(self._reconnect_delay_s)

    def _reset_connection(self) -> None:
        """Close and invalidate current serial/parser objects."""
        if self._serial is not None:
            with contextlib.suppress(serial.SerialException, OSError):
                self._serial.close()
        self._serial = None
        self._reader = None

    def close(self) -> None:
        """Close any active serial resources."""
        self._reset_connection()

    def iter_rtcm_frames(self) -> Generator[Tuple[bytes, object], None, None]:
        """Yield validated raw RTCM frames and parsed metadata forever.

        The loop tolerates malformed data, parser desync, partial reads, and
        device disconnects by continuously reconnecting and resuming parsing.
        """
        while True:
            if self._reader is None:
                self._connect()

            try:
                # RTCMReader returns (raw_bytes, parsed_message).
                raw, parsed = self._reader.read()  # type: ignore[union-attr]
                if not raw:
                    continue
                yield bytes(raw), parsed
            except (serial.SerialException, OSError, EOFError) as err:
                LOGGER.warning("[RTCM] stream disconnected: %s", err)
                LOGGER.info("[RTCM] reconnecting serial stream")
                self._reset_connection()
            except (ValueError, TypeError, RuntimeError) as err:
                LOGGER.debug("[RTCM] parser recovered from malformed bytes: %s", err)


class MAVLinkRTCMForwarder:
    """Forward RTCM frames over MAVLink GPS_RTCM_DATA with proper fragmentation."""

    def __init__(
        self,
        udp_endpoint: Optional[str],
        serial_port: Optional[str],
        serial_baud: int,
        source_system: int,
        source_component: int,
        monitor_gps: bool = False,
        gps_log_interval_s: float = 1.0,
        reconnect_delay_s: float = 2.0,
    ) -> None:
        self._udp_endpoint = udp_endpoint
        self._serial_port = serial_port
        self._serial_baud = serial_baud
        self._source_system = source_system
        self._source_component = source_component
        self._monitor_gps = monitor_gps
        self._gps_log_interval_s = gps_log_interval_s
        self._reconnect_delay_s = reconnect_delay_s

        self._mav: Optional[mavutil.mavfile] = None
        self._mav_lock = threading.Lock()
        self._frag_sequence_id = 0
        self._gps_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._last_gps_state: Optional[tuple[str, Optional[int], Optional[int], Optional[int], Optional[int]]] = None
        self._last_gps_log_ts = 0.0

    def _connect(self) -> None:
        """Connect to MAVLink transport, retrying when needed."""
        while True:
            try:
                if self._udp_endpoint:
                    LOGGER.info("[MAVLINK] connecting udp=%s", self._udp_endpoint)
                    mav_conn = mavutil.mavlink_connection(
                        self._udp_endpoint,
                        source_system=self._source_system,
                        source_component=self._source_component,
                    )
                else:
                    LOGGER.info(
                        "[MAVLINK] connecting serial=%s baud=%d",
                        self._serial_port,
                        self._serial_baud,
                    )
                    mav_conn = mavutil.mavlink_connection(
                        self._serial_port,
                        baud=self._serial_baud,
                        source_system=self._source_system,
                        source_component=self._source_component,
                    )
                with self._mav_lock:
                    self._mav = mav_conn
                LOGGER.info("[MAVLINK] connected")
                return
            except (serial.SerialException, OSError, ValueError, RuntimeError) as err:
                LOGGER.warning(
                    "[MAVLINK] connect failed: %s; retrying in %.1fs",
                    err,
                    self._reconnect_delay_s,
                )
                time.sleep(self._reconnect_delay_s)

    def _reset_connection(self) -> None:
        """Drop current MAVLink connection."""
        with self._mav_lock:
            if self._mav is not None:
                with contextlib.suppress(OSError, serial.SerialException):
                    self._mav.close()
            self._mav = None

    def _ensure_monitor_started(self) -> None:
        """Start background GPS monitor thread if enabled."""
        if not self._monitor_gps:
            return
        if self._gps_thread is not None and self._gps_thread.is_alive():
            return
        self._stop_event.clear()
        self._gps_thread = threading.Thread(target=self._gps_monitor_loop, name="gps-monitor", daemon=True)
        self._gps_thread.start()

        if self._udp_endpoint and self._udp_endpoint.startswith("udpout:"):
            LOGGER.warning(
                "[GPS] monitor enabled but mavlink endpoint '%s' is tx-only; "
                "use 'udp:' or serial to receive GPS telemetry",
                self._udp_endpoint,
            )

    def start(self) -> None:
        """Initialize optional background tasks for this forwarder."""
        self._ensure_monitor_started()

    @staticmethod
    def _fix_type_label(fix_type: Optional[int]) -> str:
        """Human-friendly label for GPS fix type values."""
        labels = {
            0: "NO_GPS",
            1: "NO_FIX",
            2: "2D_FIX",
            3: "3D_FIX",
            4: "DGPS",
            5: "RTK_FLOAT",
            6: "RTK_FIXED",
            7: "STATIC",
            8: "PPP",
        }
        if fix_type is None:
            return "UNKNOWN"
        return labels.get(fix_type, f"UNKNOWN_{fix_type}")

    def _log_gps_message(self, msg: object) -> None:
        """Log GPS status transitions from incoming MAVLink telemetry."""
        msg_type = getattr(msg, "get_type", lambda: "UNKNOWN")()
        fix_type = getattr(msg, "fix_type", None)
        sats = getattr(msg, "satellites_visible", None)
        eph = getattr(msg, "eph", None)
        epv = getattr(msg, "epv", None)

        state = (msg_type, fix_type, sats, eph, epv)
        now = time.time()
        should_log = state != self._last_gps_state or (now - self._last_gps_log_ts) >= self._gps_log_interval_s
        if not should_log:
            return

        self._last_gps_state = state
        self._last_gps_log_ts = now
        LOGGER.info(
            "[GPS] msg=%s fix=%s(%s) sats=%s eph=%s epv=%s",
            msg_type,
            fix_type,
            self._fix_type_label(fix_type),
            sats,
            eph,
            epv,
        )

    def _gps_monitor_loop(self) -> None:
        """Background loop that polls incoming MAVLink GPS telemetry."""
        while not self._stop_event.is_set():
            with self._mav_lock:
                mav_conn = self._mav

            if mav_conn is None:
                self._stop_event.wait(0.2)
                continue

            try:
                with self._mav_lock:
                    msg = mav_conn.recv_match(type=["GPS_RAW_INT", "GPS2_RAW"], blocking=False)
                if msg is not None:
                    self._log_gps_message(msg)
                else:
                    self._stop_event.wait(0.1)
            except (serial.SerialException, OSError, ValueError, RuntimeError) as err:
                LOGGER.warning("[GPS] receive error: %s", err)
                self._reset_connection()
                self._stop_event.wait(0.5)

    def close(self) -> None:
        """Close active MAVLink connection."""
        self._stop_event.set()
        if self._gps_thread is not None and self._gps_thread.is_alive():
            self._gps_thread.join(timeout=1.0)
        self._reset_connection()

    def _send_packet(self, flags: int, payload: bytes, payload_len: int, seq: int, frag_id: int) -> None:
        """Send one GPS_RTCM_DATA packet with zero-padded 180-byte payload."""
        with self._mav_lock:
            mav_conn = self._mav

        if mav_conn is None:
            self._connect()
            with self._mav_lock:
                mav_conn = self._mav

        assert mav_conn is not None  # For type-checkers.

        # MAVLink requires a fixed 180-byte array in GPS_RTCM_DATA.data.
        data = payload + (b"\x00" * (MAVLINK_RTCM_MAX_DATA_LEN - payload_len))
        with self._mav_lock:
            mav_conn.mav.gps_rtcm_data_send(flags, payload_len, data)

        if flags & 0x01:
            LOGGER.debug("[MAVLINK] sent seq=%d frag=%d len=%d", seq, frag_id, payload_len)
        else:
            LOGGER.debug("[MAVLINK] sent single len=%d", payload_len)

    def _send_with_reconnect(self, flags: int, payload: bytes, payload_len: int, seq: int, frag_id: int) -> None:
        """Send a packet and reconnect once if transport write fails."""
        try:
            self._send_packet(flags, payload, payload_len, seq, frag_id)
        except (serial.SerialException, OSError, ValueError, RuntimeError) as err:
            LOGGER.warning("[MAVLINK] send failed: %s", err)
            LOGGER.info("[MAVLINK] reconnecting output")
            self._reset_connection()
            self._send_packet(flags, payload, payload_len, seq, frag_id)

    def send_rtcm_frame(self, rtcm_frame: bytes) -> None:
        """Send one RTCM frame using GPS_RTCM_DATA with MAVLink-specified rules."""
        frame_len = len(rtcm_frame)

        if frame_len <= MAVLINK_RTCM_MAX_DATA_LEN:
            # Non-fragmented packet: flags must be exactly zero.
            self._send_with_reconnect(flags=0, payload=rtcm_frame, payload_len=frame_len, seq=0, frag_id=0)
            return

        if frame_len > MAVLINK_RTCM_MAX_FRAME_LEN:
            LOGGER.warning(
                "[MAVLINK] dropping RTCM frame len=%d (max supported %d)",
                frame_len,
                MAVLINK_RTCM_MAX_FRAME_LEN,
            )
            return

        num_fragments = (frame_len + MAVLINK_RTCM_MAX_DATA_LEN - 1) // MAVLINK_RTCM_MAX_DATA_LEN
        sequence_id = self._frag_sequence_id
        self._frag_sequence_id = (self._frag_sequence_id + 1) % 32

        for frag_id in range(num_fragments):
            start = frag_id * MAVLINK_RTCM_MAX_DATA_LEN
            end = min(start + MAVLINK_RTCM_MAX_DATA_LEN, frame_len)
            chunk = rtcm_frame[start:end]
            chunk_len = len(chunk)

            # flags bit layout for fragmented packets:
            # bit0   = 1 (fragmented)
            # bits1-2= fragment ID
            # bits3-7= sequence ID
            flags = 0x01 | ((frag_id & 0x03) << 1) | ((sequence_id & 0x1F) << 3)
            self._send_with_reconnect(
                flags=flags,
                payload=chunk,
                payload_len=chunk_len,
                seq=sequence_id,
                frag_id=frag_id,
            )


def _rtcm_message_type(parsed_msg: object) -> str:
    """Extract RTCM message type from pyrtcm parsed message object."""
    # pyrtcm objects usually provide `identity` (e.g. "1005").
    identity = getattr(parsed_msg, "identity", None)
    if identity is not None:
        return str(identity)

    # Fallback to DF002 if identity is unavailable.
    df002 = getattr(parsed_msg, "DF002", None)
    if df002 is not None:
        return str(df002)

    return "unknown"


def _configure_logging(debug: bool) -> None:
    """Initialize structured logging."""
    level = logging.DEBUG if debug else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def _parse_args(argv: Optional[list[str]] = None) -> BridgeConfig:
    """Parse command-line arguments into a strongly typed config object."""
    parser = argparse.ArgumentParser(
        description="Bridge RTCM v3 from mixed serial stream to MAVLink GPS_RTCM_DATA."
    )
    parser.add_argument("--rtcm-port", required=True, help="RTCM input serial port (e.g. /dev/ttyUSB0)")
    parser.add_argument("--rtcm-baud", required=True, type=int, help="RTCM input serial baudrate")

    out_group = parser.add_mutually_exclusive_group(required=True)
    out_group.add_argument(
        "--mavlink-udp",
        help="MAVLink UDP endpoint (e.g. udpout:127.0.0.1:14550)",
    )
    out_group.add_argument("--mavlink-serial", help="MAVLink output serial port (e.g. /dev/ttyACM0)")

    parser.add_argument(
        "--mavlink-baud",
        type=int,
        default=115200,
        help="MAVLink serial baudrate (used only with --mavlink-serial)",
    )
    parser.add_argument("--source-system", type=int, default=250, help="MAVLink source system ID")
    parser.add_argument("--source-component", type=int, default=191, help="MAVLink source component ID")
    parser.add_argument(
        "--monitor-gps",
        action="store_true",
        help="Log incoming MAVLink GPS status (GPS_RAW_INT/GPS2_RAW) from the same connection",
    )
    parser.add_argument(
        "--gps-log-interval",
        type=float,
        default=1.0,
        help="Minimum seconds between repeated GPS status logs when values are unchanged",
    )
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")

    args = parser.parse_args(argv)
    return BridgeConfig(
        rtcm_port=args.rtcm_port,
        rtcm_baud=args.rtcm_baud,
        mavlink_udp=args.mavlink_udp,
        mavlink_serial=args.mavlink_serial,
        mavlink_baud=args.mavlink_baud,
        source_system=args.source_system,
        source_component=args.source_component,
        debug=args.debug,
        monitor_gps=args.monitor_gps,
        gps_log_interval_s=max(args.gps_log_interval, 0.1),
    )


def _install_signal_handlers() -> None:
    """Enable graceful termination for SIGINT/SIGTERM."""

    def _handle_signal(signum: int, _frame: object) -> None:
        raise KeyboardInterrupt(f"signal {signum}")

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)


def run(config: BridgeConfig) -> int:
    """Run the bridge until interrupted."""
    _configure_logging(config.debug)
    _install_signal_handlers()

    reader = RTCMSerialReader(
        port=config.rtcm_port,
        baudrate=config.rtcm_baud,
        reconnect_delay_s=config.reconnect_delay_s,
        timeout_s=config.serial_timeout_s,
    )
    forwarder = MAVLinkRTCMForwarder(
        udp_endpoint=config.mavlink_udp,
        serial_port=config.mavlink_serial,
        serial_baud=config.mavlink_baud,
        source_system=config.source_system,
        source_component=config.source_component,
        monitor_gps=config.monitor_gps,
        gps_log_interval_s=config.gps_log_interval_s,
        reconnect_delay_s=config.reconnect_delay_s,
    )
    forwarder.start()

    try:
        for raw_frame, parsed_msg in reader.iter_rtcm_frames():
            if config.debug:
                LOGGER.debug("[RTCM] type=%s len=%d", _rtcm_message_type(parsed_msg), len(raw_frame))
            forwarder.send_rtcm_frame(raw_frame)
    except KeyboardInterrupt:
        LOGGER.info("Shutting down bridge")
        return 0
    except (serial.SerialException, OSError, ValueError, RuntimeError):
        LOGGER.exception("Fatal bridge error")
        return 1
    finally:
        reader.close()
        forwarder.close()


def main() -> int:
    """CLI entrypoint."""
    config = _parse_args()
    return run(config)


if __name__ == "__main__":
    sys.exit(main())
