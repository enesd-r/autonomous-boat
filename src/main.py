#!/usr/bin/env python3
import os
import time
import csv
from datetime import datetime
from pymavlink import mavutil

LOG_DIR = "/home/pi/autonomous-boat/logs"      # MANUEL oluşturacaksın
SERIAL_DEV = "/dev/ttyAMA0"                     # Pixhawk UART (TELEM2)
BAUD = 57600                                    # ArduPilot default UART
AUTO_ARM = False                                # Fail-safe: ARM manuel yapılacak
LOITER_TIME = 20                                # Her WP sonrası bekleme süresi
HEARTBEAT_TIMEOUT = 10
RETRY_CONNECT_SECONDS = 2

WAYPOINTS = [
    (41.00234, 29.01245),
    (41.00290, 29.01320),
    (41.00200, 29.01350),
    (41.00160, 29.01290),
]


def open_logfile():
    fname = os.path.join(LOG_DIR, f"mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    f = open(fname, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["timestamp", "lat", "lon", "mode", "event", "note"])
    f.flush()
    return f, writer


def connect_pixhawk():
    while True:
        try:
            master = mavutil.mavlink_connection(SERIAL_DEV, baud=BAUD)
            master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT)
            return master
        except Exception:
            time.sleep(RETRY_CONNECT_SECONDS)


def get_latest_global_pos(master):
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if msg:
        return msg.lat / 1e7, msg.lon / 1e7
    return None, None


def log_event(writer, master, event, note=""):
    lat, lon = get_latest_global_pos(master)
    if lat is None:
        lat = 0
        lon = 0
    mode = getattr(master, "flightmode", "N/A")
    writer.writerow([datetime.now().isoformat(), lat, lon, mode, event, note])
    try:
        writer.flush()
    except:
        pass


def goto(master, lat, lon, alt=0):
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        int(0b110111111000),
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )


def check_for_emergency(master):
    msg = master.recv_match(type="RC_CHANNELS", blocking=False)
    if not msg:
        return False
    chan7 = getattr(msg, "chan7_raw", None)
    if chan7 is None:
        return False
    if chan7 < 1200:
        return True
    return False


def main():
    f, writer = open_logfile()
    master = connect_pixhawk()
    log_event(writer, master, "CONNECTED")

    try:
        master.set_mode_apm("GUIDED")
        time.sleep(1)
    except:
        log_event(writer, master, "GUIDED_FAIL")

    if AUTO_ARM:
        try:
            master.arducopter_arm()
            master.motors_armed_wait()
            log_event(writer, master, "ARMED_AUTO")
        except:
            log_event(writer, master, "ARM_FAIL")
            return
    else:
        log_event(writer, master, "WAITING_MANUAL_ARM")

    try:
        for i, (lat, lon) in enumerate(WAYPOINTS, start=1):
            if check_for_emergency(master):
                log_event(writer, master, "EMERGENCY_STOP", f"before GN{i}")
                break

            log_event(writer, master, f"GN{i}_START", f"target={lat},{lon}")
            goto(master, lat, lon, 0)

            for t in range(LOITER_TIME):
                if check_for_emergency(master):
                    log_event(writer, master, "EMERGENCY_STOP", f"during GN{i}")
                    raise KeyboardInterrupt
                log_event(writer, master, f"GN{i}_LOITER", f"t={t}s")
                time.sleep(1)

            log_event(writer, master, f"GN{i}_REACHED")

        log_event(writer, master, "MISSION_COMPLETE", "sending RTL")
        try:
            master.set_mode_apm("RTL")
            log_event(writer, master, "RTL_CMD_SENT")
        except:
            log_event(writer, master, "RTL_FAIL")

        time.sleep(10)

    except KeyboardInterrupt:
        log_event(writer, master, "INTERRUPT_RTL")
        try:
            master.set_mode_apm("RTL")
        except:
            pass

    finally:
        log_event(writer, master, "MISSION_ENDED")
        f.close()
        master.close()


if __name__ == "__main__":
    main()
