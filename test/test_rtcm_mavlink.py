import os

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil

mav = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
print("Waiting for GPS_RTCM_DATA ...")
while True:
    msg = mav.recv_match(type="GPS_RTCM_DATA", blocking=True, timeout=5)
    if msg:
        print(f"flags={msg.flags:#04x} len={msg.len} data={bytes(msg.data[:8]).hex()}")
    else:
        print("(timeout - no message received)")
