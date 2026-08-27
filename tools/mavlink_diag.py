#!/usr/bin/env python3
"""mavlink_diag.py — diagnostic that matches MiniHawk's connection string."""

from pymavlink import mavutil
import time

print("[*] Binding using MiniHawk's syntax: udp:127.0.0.1:14550")
m = mavutil.mavlink_connection('udp:127.0.0.1:14550')

if hasattr(m, 'port'):
    import socket
    try:
        print(f"    Local bind: {m.port.getsockname()}")
    except Exception as e:
        print(f"    Local bind error: {e}")
    print(f"    Connected to: {m.port.getpeername() if hasattr(m.port, 'getpeername') else 'N/A'}")

print("[*] Waiting 15 seconds for any MAVLink packet...")
print("    (Keep Mission Planner mirror running during this test)\n")

count = 0
heartbeat_seen = False
start = time.time()
while time.time() - start < 15:
    msg = m.recv_match(blocking=False)
    if msg:
        count += 1
        t = msg.get_type()
        sysid = msg.get_srcSystem()
        if t == "HEARTBEAT":
            heartbeat_seen = True
            print(f"  [HEARTBEAT] type={msg.type} autopilot={msg.autopilot} sysid={sysid}")
        elif count <= 3:
            print(f"  [{count}] type={t:20} sysid={sysid}")
    else:
        time.sleep(0.01)

print(f"\n[*] Total packets in 15s: {count}")
if count == 0:
    print("[WARNING] Zero packets received.")
    print("  -> Mission Planner UDP Mirror only forwards autopilot traffic.")
    print("  -> If NO DRONE is connected to MP, THIS IS EXPECTED.")
    print("  -> Connect the drone via telemetry cable or use fake_mavlink_sender.py")
elif heartbeat_seen:
    print("[OK] Heartbeat(s) received. MiniHawk should work.")
else:
    print("[INFO] Packets arrived but no heartbeat. MiniHawk may still work.")
