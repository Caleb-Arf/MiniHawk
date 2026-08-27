#!/usr/bin/env python3
"""
fake_mavlink_sender.py

Sends synthetic MAVLink messages to MiniHawk (or any MAVLink listener) on UDP.
Generates realistic fake telemetry for testing the dashboard without real hardware.

Flies a parametric circuit (figure-8 pattern over Centennial Olympic Park, Atlanta)
at ~5 m/s with realistic AHRS, GPS fix, battery and environmental sensors.

Usage:
    python test/fake_mavlink_sender.py
    python test/fake_mavlink_sender.py --armed --noise --rate 5
"""

import argparse
import math
import random
import socket
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink

# --- Mission / flight geometry ------------------------------------------------
CENTER_LAT = 9.9396          # Gaslab UCR
CENTER_LON = -84.0424
CENTER_ALT = 1300.0             # metres AMSL
FLIGHT_RADIUS_M = 600.0       # roughly 0.005 deg at this latitude
GROUND_SPEED_MPS = 5.0
# Earth's approximate metres-per-degree at this latitude
M_PER_DEG_LAT = 111_320.0
M_PER_DEG_LON = 111_320.0 * math.cos(math.radians(CENTER_LAT))


def create_mavlink():
    """Return a MAVLink encode instance."""
    mav = mavlink.MAVLink(None)
    mav.srcSystem = 1
    mav.srcComponent = 1
    return mav


def send_heartbeat(sock, target, mav, armed=False):
    base_mode = (
        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        if armed else 0
    ) | mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
    # type=2 (MAV_TYPE_QUADROTOR), autopilot=12 (MAV_AUTOPILOT_PX4), state=4 (active)
    msg = mav.heartbeat_encode(2, 12, base_mode, 0, 4, 3)
    buf = msg.pack(mav)
    sock.sendto(buf, target)


def send_global_position_int(sock, target, mav, lat_deg, lon_deg, alt_m, rel_alt_m,
                            vx_ms, vy_ms, vz_ms, hdg_deg):
    """Send GLOBAL_POSITION_INT with full kinematics."""
    lat_e7 = int(lat_deg * 1e7)
    lon_e7 = int(lon_deg * 1e7)
    alt_mm = int(alt_m * 1000)
    rel_alt_mm = int(rel_alt_m * 1000)
    msg = mav.global_position_int_encode(
        0, lat_e7, lon_e7, alt_mm, rel_alt_mm,
        int(vx_ms * 100), int(vy_ms * 100), int(vz_ms * 100),
        int(hdg_deg * 100)
    )
    buf = msg.pack(mav)
    sock.sendto(buf, target)


def send_gps_raw_int(sock, target, mav, lat_deg, lon_deg, alt_m, hdg_deg, vel_ms, satellites=12):
    """Send GPS_RAW_INT with a fake 3D-fix and HDOP."""
    msg = mav.gps_raw_int_encode(
        0,                              # time_usec (unused)
        3,                              # fix_type: 3 = 3D-fix
        int(lat_deg * 1e7),
        int(lon_deg * 1e7),
        int(alt_m * 1000),
        200, 200,                       # eph, epv (1.5 m)
        int(vel_ms * 100),              # ground speed
        int(hdg_deg * 100),             # course over ground
        satellites
    )
    buf = msg.pack(mav)
    sock.sendto(buf, target)


def send_attitude(sock, target, mav, roll_rad, pitch_rad, yaw_rad,
                  rollspeed, pitchspeed, yawspeed):
    msg = mav.attitude_encode(0, roll_rad, pitch_rad, yaw_rad, rollspeed, pitchspeed, yawspeed)
    buf = msg.pack(mav)
    sock.sendto(buf, target)


def send_vfr_hud(sock, target, mav, airspeed_ms, groundspeed_ms, heading, throttle_pct,
                 alt_m, climb_ms):
    msg = mav.vfr_hud_encode(airspeed_ms, groundspeed_ms, heading, throttle_pct,
                             int(alt_m), climb_ms)
    buf = msg.pack(mav)
    sock.sendto(buf, target)


def send_named_float(sock, target, mav, name: str, value: float):
    time_boot_ms = int(time.time() * 1000) % (2 ** 32)
    if isinstance(name, str):
        name = name.encode("ascii")
    msg = mav.named_value_float_encode(time_boot_ms, name, value)
    buf = msg.pack(mav)
    sock.sendto(buf, target)


def flight_state(t):
    """
    Return a realistic flight state for time *t* (seconds).

    Path is a horizontal figure-8 (Lissajous) so latitude and longitude trace
    a smooth closed loop around CENTER_LAT / CENTER_LON.
    """
    # Angular rate depends on desired tangential speed.
    # Circumference of a circle with radius a is 2*pi*a.
    # For a figure-8 (two loops) we treat one full cycle as 2*pi.
    angular_speed = GROUND_SPEED_MPS / FLIGHT_RADIUS_M

    theta = t * angular_speed

    # Lissajous figure-8 scaled to degrees
    dlat = (FLIGHT_RADIUS_M * math.sin(theta)) / M_PER_DEG_LAT
    dlon = (FLIGHT_RADIUS_M * math.sin(2 * theta) * 0.5) / M_PER_DEG_LON

    lat = CENTER_LAT + dlat
    lon = CENTER_LON + dlon

    # Altitude: gentle hover + small sine wave
    alt = CENTER_ALT + 5.0 * math.sin(theta * 0.5)

    # Heading: tangent to the path (derivative of position)
    dlat_dt = (FLIGHT_RADIUS_M * math.cos(theta) * angular_speed) / M_PER_DEG_LAT
    dlon_dt = (FLIGHT_RADIUS_M * math.cos(2 * theta) * angular_speed) / M_PER_DEG_LON
    hdg = (math.degrees(math.atan2(dlon_dt, dlat_dt)) + 360) % 360

    # Velocity vector
    vx = GROUND_SPEED_MPS * math.cos(math.radians(hdg))
    vy = GROUND_SPEED_MPS * math.sin(math.radians(hdg))
    vz = 2.5 * math.cos(theta * 0.5)   # slow climb/descent

    # Attitude: slight bank into turns, small pitch for forward flight
    yaw = math.radians(hdg)
    # Bank angle proportional to lateral acceleration (coordinated turn approx)
    turn_rate = (math.radians(hdg) - math.radians((hdg - 0.5) % 360))
    roll = math.atan(turn_rate * GROUND_SPEED_MPS / 9.81)
    # Limit / smooth roll
    roll = max(-0.35, min(0.35, roll + 0.05 * math.sin(theta)))
    pitch = 0.05 + 0.03 * math.sin(theta * 0.75)

    return lat, lon, alt, vx, vy, vz, hdg, roll, pitch, yaw


def main():
    parser = argparse.ArgumentParser(
        description="Fake MAVLink telemetry sender for MiniHawk"
    )
    parser.add_argument("--host", default="127.0.0.1",
                        help="Target IP address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=14550,
                        help="Target UDP port (default: 14550)")
    parser.add_argument("--rate", type=float, default=2.0,
                        help="Sensor telemetry rate in Hz (default: 2)")
    parser.add_argument("--armed", action="store_true",
                        help="Set ARMED bit in MAVLink HEARTBEAT")
    parser.add_argument("--noise", action="store_true",
                        help="Add random noise to CO2, humidity and temperature values")
    args = parser.parse_args()

    target = (args.host, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    mav = create_mavlink()

    print(f"[INFO] Sending fake MAVLink to udp:{args.host}:{args.port}")
    print(f"[INFO] Telemetry rate: {args.rate} Hz | Armed: {args.armed}")
    print(f"[INFO] Flight pattern: figure-8 around ({CENTER_LAT}, {CENTER_LON})")
    print("[INFO] Press Ctrl+C to stop.\n")

    tick = 0
    dt = 1.0 / args.rate

    try:
        while True:
            t = time.time()
            tick += 1

            lat, lon, alt, vx, vy, vz, hdg, roll, pitch, yaw = flight_state(t)

            # --- 1 Hz messages ------------------------------------------------
            if tick % max(1, int(args.rate)) == 0:
                send_heartbeat(sock, target, mav, armed=args.armed)
                send_global_position_int(
                    sock, target, mav, lat, lon, alt, alt,
                    vx, vy, vz, hdg
                )
                send_gps_raw_int(sock, target, mav, lat, lon, alt, hdg,
                                 math.hypot(vx, vy), satellites=12)
                send_attitude(
                    sock, target, mav,
                    roll, pitch, yaw,
                    rollspeed=0.02 * math.sin(t),
                    pitchspeed=0.01 * math.cos(t),
                    yawspeed=0.05 * math.sin(t * 0.5)
                )
                send_vfr_hud(
                    sock, target, mav,
                    airspeed_ms=math.hypot(vx, vy, vz) + random.uniform(-0.2, 0.2),
                    groundspeed_ms=math.hypot(vx, vy),
                    heading=int(hdg),
                    throttle_pct=45 + int(20 * math.sin(t * 0.3)),
                    alt_m=alt,
                    climb_ms=vz
                )

            # --- Sensor telemetry (NAMED_VALUE_FLOAT) -----------------------
            # Match the approximate ranges & behaviour seen in test/sample.csv
            # TEMP  ~22.2  (very flat, tiny drift)
            # HUM   ~17.5 -> 19.5 (slow upward drift + small ripple)
            # SINE  ±5.0  (smooth sinusoid)
            # CO2   ~400  (larger amplitude for scatter plot visual)
            co2_base  = 4400.0 + 200.0 * math.sin(t * 0.2)
            hum_base  = 17.5 + 0.002 * t + 0.3 * math.sin(t * 0.05)
            temp_base = 22.2 + 0.15 * math.sin(t * 0.03 + 1.0)
            sine_base = 5.0 * math.sin(t * 0.063)

            # H2S  ~0-100 ppm  (4-20mA sensor, full scale 100 ppm)
            # SO2  ~0-50 ppm   (4-20mA sensor, full scale 50 ppm)
            # CO2_FINE ~0-3000 ppm (4-20mA sensor, full scale 3000 ppm)
            h2s_base = 20.0 + 15.0 * math.sin(t * 0.15) + 10.0 * math.sin(t * 0.08)
            so2_base = 5.0 + 3.0 * math.sin(t * 0.12) + 2.0 * math.sin(t * 0.07)
            co2fine_base = 1200.0 + 400.0 * math.sin(t * 0.1) + 200.0 * math.sin(t * 0.06)

            if args.noise:
                co2_base  += random.uniform(-10.0, 10.0)
                hum_base  += random.uniform(-0.1, 0.1)
                temp_base += random.uniform(-0.05, 0.05)
                sine_base += random.uniform(-0.05, 0.05)
                h2s_base  += random.uniform(-2.0, 2.0)
                so2_base  += random.uniform(-0.5, 0.5)
                co2fine_base += random.uniform(-30.0, 30.0)

            send_named_float(sock, target, mav, "CO2", co2_base)
            send_named_float(sock, target, mav, "HUMIDITY", hum_base)
            send_named_float(sock, target, mav, "TEMP", temp_base)
            send_named_float(sock, target, mav, "H2S", h2s_base)
            send_named_float(sock, target, mav, "SO2", so2_base)
            send_named_float(sock, target, mav, "CO2_FINE", co2fine_base)
            send_named_float(sock, target, mav, "SINE", sine_base)

            time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[INFO] Fake MAVLink sender stopped.")
        sock.close()


if __name__ == "__main__":
    main()
