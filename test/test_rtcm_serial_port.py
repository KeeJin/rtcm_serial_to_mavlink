import serial
from pyrtcm import RTCMReader
s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
rdr = RTCMReader(s)
while True:
    raw, msg = rdr.read()
    if msg: print(f'type={msg.identity} len={len(raw)}')