# rtcm_serial_to_mavlink

Bridge RTCM v3 correction data from a mixed serial byte stream into MAVLink
`GPS_RTCM_DATA` messages (ID 233) for ArduPilot.  Runs on any Linux system
with Python 3 and a serial port; the included systemd unit deploys it as a
persistent background service.

---

## Table of contents

1. [How it works](#how-it-works)
2. [Requirements](#requirements)
3. [Installation](#installation)
4. [CLI usage](#cli-usage)
5. [CLI reference](#cli-reference)
6. [Verifying the pipeline](#verifying-the-pipeline)
7. [systemd service](#systemd-service)
8. [MAVLink fragmentation](#mavlink-fragmentation)
9. [Reliability and reconnect behaviour](#reliability-and-reconnect-behaviour)
10. [Repository layout](#repository-layout)

---

## How it works

```
Serial port  ──►  RTCMSerialReader  ──►  MAVLinkRTCMForwarder  ──►  ArduPilot
(mixed binary)   (pyrtcm parser)        (GPS_RTCM_DATA / MAVLink 2)
```

`RTCMSerialReader` owns the pyserial connection and feeds raw bytes into
`pyrtcm.RTCMReader`.  The parser performs stream synchronisation and CRC24Q
validation internally, automatically recovering from garbage bytes and
desync.  Each validated raw RTCM frame is passed unchanged to
`MAVLinkRTCMForwarder`, which applies the MAVLink fragmentation rules and
transmits one or more `GPS_RTCM_DATA` packets over UDP or serial.

---

## Requirements

| Package | Role |
|---|---|
| `pyserial >= 3.5` | Serial port I/O |
| `pymavlink >= 2.4.39` | MAVLink 2 framing and transport |
| `pyrtcm >= 1.1.0` | RTCM v3 stream parsing and CRC validation |

Install all dependencies:

```bash
pip install -r requirements.txt
```

Or individually:

```bash
pip install "pyserial>=3.5,<4.0" "pymavlink>=2.4.39,<3.0" "pyrtcm>=1.1.0,<2.0"
```

---

## Installation

```bash
# 1. Clone the repository
git clone <repo-url> ~/rtcm_serial_to_mavlink
cd ~/rtcm_serial_to_mavlink

# 2. Create a virtual environment and install dependencies
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

# 3. (Optional) Add your user to the dialout group for serial port access
sudo usermod -aG dialout $USER
# Log out and back in for the group change to take effect
```

---

## CLI usage

**UDP output (most common — ArduPilot listening on UDP):**

```bash
python rtcm_bridge.py \
  --rtcm-port /dev/ttyACM0 \
  --rtcm-baud 115200 \
  --mavlink-udp udpout:127.0.0.1:14550
```

**Serial output (direct UART to flight controller):**

```bash
python rtcm_bridge.py \
  --rtcm-port /dev/ttyUSB0 \
  --rtcm-baud 460800 \
  --mavlink-serial /dev/ttyACM0 \
  --mavlink-baud 921600
```

**With debug logging:**

```bash
python rtcm_bridge.py \
  --rtcm-port /dev/ttyACM0 \
  --rtcm-baud 115200 \
  --mavlink-udp udpout:127.0.0.1:14550 \
  --debug
```

Debug output example:

```
2026-05-26 10:00:02 DEBUG [RTCM] type=1005 len=43
2026-05-26 10:00:02 DEBUG [MAVLINK] sent single len=43
2026-05-26 10:00:02 DEBUG [RTCM] type=1074 len=327
2026-05-26 10:00:02 DEBUG [MAVLINK] sent seq=0 frag=0 len=180
2026-05-26 10:00:02 DEBUG [MAVLINK] sent seq=0 frag=1 len=147
```

---

## CLI reference

| Flag | Required | Default | Description |
|---|---|---|---|
| `--rtcm-port` | yes | — | Serial port carrying the RTCM stream (e.g. `/dev/ttyACM0`) |
| `--rtcm-baud` | yes | — | Baud rate of the RTCM serial port |
| `--mavlink-udp` | one of | — | MAVLink UDP endpoint (e.g. `udpout:127.0.0.1:14550`) |
| `--mavlink-serial` | one of | — | MAVLink output serial port (e.g. `/dev/ttyACM0`) |
| `--mavlink-baud` | no | `115200` | Baud rate for `--mavlink-serial` |
| `--source-system` | no | `250` | MAVLink source system ID |
| `--source-component` | no | `191` | MAVLink source component ID |
| `--debug` | no | off | Enable DEBUG-level logging |

`--mavlink-udp` and `--mavlink-serial` are mutually exclusive; exactly one is required.

---

## Verifying the pipeline

### Stage 1 — Confirm RTCM is arriving on the serial port

```bash
python test/test_rtcm_serial_port.py
```

Expected output:

```
type=1005 len=43
type=1074 len=127
```

If nothing appears within a few seconds the port or baud rate is wrong.

### Stage 2 — Confirm the bridge is parsing and sending

Run the bridge with `--debug` and watch for both `[RTCM]` and `[MAVLINK] sent` lines (see [debug output example](#cli-usage) above).

### Stage 3 — Confirm MAVLink packets are produced

In a second terminal, run the listener while the bridge is running:

```bash
python test/test_rtcm_mavlink.py
```

Expected output:

```
Waiting for GPS_RTCM_DATA ...
flags=0x00 len=43  data=d3001a3ed7d30000
flags=0x01 len=180 data=...
flags=0x05 len=147 data=...
```

`flags=0x00` is an unfragmented packet.  
`flags & 0x01 == 1` indicates a fragment; bits 1–2 encode the fragment index.

### Stage 4 — End-to-end with ArduPilot

Check `GPS_STATUS` in QGroundControl or Mission Planner.  When RTCM corrections are received and applied, the GPS fix type will show **RTK Float** or **RTK Fixed**.

---

## systemd service

Files are in the `service/` directory:

```
service/
├── rtcm_bridge.service   # systemd unit template
└── install_service.sh    # installer script
```

### Deploy on the Orin NX

```bash
cd ~/rtcm_serial_to_mavlink

# Make the installer executable
chmod +x service/install_service.sh

# Install with defaults (/dev/ttyACM0 @ 115200 → udpout:127.0.0.1:14550)
./service/install_service.sh
```

The installer:
1. Detects the current user and home directory automatically.
2. Templates `__USER__` and `__HOME__` placeholders into the unit file.
3. Copies the resolved unit to `/etc/systemd/system/rtcm_bridge.service`.
4. Runs `systemctl daemon-reload` and `systemctl enable rtcm_bridge`.
5. Warns if the user is not in the `dialout` group.

### Override hardware configuration

Pass environment variables before the script to customise the unit file
without editing it:

```bash
RTCM_PORT=/dev/ttyUSB0 \
RTCM_BAUD=460800 \
MAVLINK_TARGET=udpout:192.168.1.10:14550 \
./service/install_service.sh
```

| Variable | Default |
|---|---|
| `RTCM_PORT` | `/dev/ttyACM0` |
| `RTCM_BAUD` | `115200` |
| `MAVLINK_TARGET` | `udpout:127.0.0.1:14550` |

### Service management

```bash
# Start immediately
sudo systemctl start rtcm_bridge

# Check status
sudo systemctl status rtcm_bridge

# Follow live logs
journalctl -u rtcm_bridge -f

# Stop
sudo systemctl stop rtcm_bridge

# Disable autostart
sudo systemctl disable rtcm_bridge

# Uninstall completely
sudo systemctl disable --now rtcm_bridge
sudo rm /etc/systemd/system/rtcm_bridge.service
sudo systemctl daemon-reload
```

### Serial device dependency

The unit declares `Wants=` and `After=` on the systemd device unit for the
configured RTCM serial port (e.g. `dev-ttyACM0.device` for `/dev/ttyACM0`).
The install script computes this automatically using `systemd-escape --path`.

- `After=` ensures systemd does not start the bridge until the kernel has
  enumerated the device, avoiding a failed open on boot.
- `Wants=` (not `Requires=`) means a missing device at boot delays the start
  but does not permanently prevent it.  Once the device appears systemd will
  start the service.
- Mid-run disconnects (e.g. cable pulled) are handled by the bridge's own
  reconnect loop — the service does not need to be restarted by systemd for
  those events.

### Restart policy

The service restarts automatically on any non-zero exit.  A burst limiter
prevents a restart storm: if the service fails more than 5 times in 60 seconds
systemd backs off.  Serial reconnect and MAVLink reconnect are also handled
internally by the bridge without requiring a service restart.

---

## MAVLink fragmentation

`GPS_RTCM_DATA` carries at most 180 bytes of payload per packet.  RTCM frames
larger than 180 bytes are split into up to 4 fragments.

**Flags field bit layout:**

| Bits | Field | Description |
|---|---|---|
| 0 | fragmented | `1` if this message is part of a fragmented sequence, `0` otherwise |
| 1–2 | fragment ID | Index of this fragment within the sequence (0–3) |
| 3–7 | sequence ID | Per-message counter, increments modulo 32 for each fragmented RTCM frame |

**Single packet** (RTCM frame ≤ 180 bytes):
- `flags = 0x00`
- `len` = actual frame length
- `data` zero-padded to 180 bytes

**Fragmented packet** (RTCM frame 181–720 bytes):
- `flags = 1 | (frag_id << 1) | (sequence_id << 3)`
- `len` = size of this fragment (last fragment may be shorter than 180)
- `data` zero-padded to 180 bytes

Frames exceeding 720 bytes (4 × 180) are dropped with a warning log; this
does not occur with any standard RTCM message set.

---

## Reliability and reconnect behaviour

### Serial input reconnect

If the RTCM serial device disconnects or produces a read error:
1. The serial port and `RTCMReader` are closed and discarded.
2. A reconnect loop retries `serial.Serial()` every 2 seconds until the device reappears.
3. Once reconnected, a fresh `RTCMReader` is created and parsing resumes from the new stream start.
4. Malformed bytes and parser desync between reconnects are silently skipped.

### MAVLink output reconnect

If the MAVLink transport write fails:
1. The connection is closed.
2. A single reconnect attempt is made immediately.
3. The failed packet is retransmitted after reconnect.
4. If the second attempt also fails the exception propagates (service restarts via systemd).

### Other robustness properties

- `KeyboardInterrupt` and `SIGTERM` trigger a clean shutdown that closes both connections.
- The `pyrtcm` parser handles stream desync and CRC failures internally; no raw byte manipulation is done in this code.
- Raw RTCM bytes are forwarded without re-serialisation to avoid introducing errors.

---

## Repository layout

```
rtcm_serial_to_mavlink/
├── rtcm_bridge.py          # Main bridge program
├── requirements.txt        # Python dependencies
├── service/
│   ├── rtcm_bridge.service # systemd unit template
│   └── install_service.sh  # Service installer
└── test/
    ├── test_rtcm_serial_port.py  # Stage 1: verify RTCM input
    └── test_rtcm_mavlink.py      # Stage 3: verify MAVLink output
```


Example usage commands:
python rtcm_bridge.py --rtcm-port /dev/ttyUSB0 --rtcm-baud 115200 --mavlink-udp udpout:127.0.0.1:14550

python rtcm_bridge.py --rtcm-port /dev/ttyUSB0 --rtcm-baud 460800 --mavlink-serial /dev/ttyACM0 --mavlink-baud 921600