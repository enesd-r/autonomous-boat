#!/usr/bin/env python3
"""
Main mission script.
- Goes to a list of waypoints (lat, lon).
- Logs GPS + events to a timestamped CSV in LOG_DIR.
- Optionally can AUTO_ARM (default False) -- use with extreme caution.
- RC channel 7 is used as an emergency STOP (switch off).
"""
import os
import time
import csv
from datetime import datetime
from pymavlink import mavutil

# -------------- CONFIG --------------
LOG_DIR = "/home/pi/autonomous-boat/logs"   # final log directory
SERIAL_DEV = "/dev/ttyAMA0"                 # change if necessary
BAUD = 115200                               # match to Pixhawk TELEM2 baud
WAYPOINTS = [
    (41.00234, 29.01245),
    (41.00290, 29.01320),
    (41.00200, 29.01350),
    (41.00160, 29.01290),
]
LOITER_TIME = 20         # seconds to wait after sending goto
HEARTBEAT_TIMEOUT = 10
AUTO_ARM = False         # D E F A U L T  =  False  (safe)
RETRY_CONNECT_SECONDS = 2
# ------------------------------------

os.makedirs(LOG_DIR, exist_ok=True)


def open_logfile():
    fname = os.path.join(LOG_DIR, f"mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    f = open(fname, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["timestamp", "lat", "lon", "mode", "event", "note"])
    f.flush()
    return f, writer, fname


def connect_pixhawk(dev, baud, timeout=HEARTBEAT_TIMEOUT):
    """Try to open mavlink connection and wait for heartbeat."""
    while True:
        try:
            m = mavutil.mavlink_connection(dev, baud=baud)
            print(f"[INFO] Waiting for heartbeat on {dev} @ {baud}...")
            m.wait_heartbeat(timeout=timeout)
            print("[INFO] Heartbeat received.")
            return m
        except Exception as e:
            print(f"[WARN] Connection error: {e}. Retrying in {RETRY_CONNECT_SECONDS}s...")
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
    except Exception:
        pass


def goto(master, lat, lon, alt=0):
    """Send SET_POSITION_TARGET_GLOBAL_INT to Pixhawk."""
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        int(0b110111111000),       # type mask: only pos
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,  # vx,vy,vz
        0, 0, 0,  # afx, afy, afz
        0, 0      # yaw, yaw_rate
    )


def check_for_emergency(master):
    """Check RC_CHANNELS for channel 7 switch (example logic).
       Return True if emergency stop triggered."""
    msg = master.recv_match(type="RC_CHANNELS", blocking=False)
    if not msg:
        return False
    # handle missing channel gracefully
    chan7 = getattr(msg, "chan7_raw", None)
    if chan7 is None:
        return False
    # Example threshold: < 1200 means switched to STOP
    if chan7 < 1200:
        return True
    return False


def main():
    f, writer, fname = open_logfile()
    print(f"[INFO] Logging to {fname}")

    master = connect_pixhawk(SERIAL_DEV, BAUD)
    # keep master in scope for logging functions

    # initial log
    log_event(writer, master, "CONNECTED")

    # set guided mode (note: depends on AP version)
    try:
        master.set_mode_apm("GUIDED")
        time.sleep(1)
    except Exception as e:
        print(f"[WARN] Failed to set GUIDED mode: {e}")

    # Auto-arm if explicitly enabled (DANGEROUS)
    if AUTO_ARM:
        try:
            print("[WARN] AUTO_ARM is True -> attempting to arm (use with care).")
            master.arducopter_arm()
            master.motors_armed_wait()
            log_event(writer, master, "ARMED", "auto_arm=True")
        except Exception as e:
            print(f"[ERROR] Auto-arm failed: {e}")
            log_event(writer, master, "ARM_FAIL", str(e))
            # continue but do not proceed with mission if not armed
            if not getattr(master, "motors_armed", False):
                print("[ERROR] Not armed; exiting for safety.")
                f.close()
                master.close()
                return
    else:
        log_event(writer, master, "AWAITING_ARM", "auto_arm=False")

    try:
        # Mission loop: visit each waypoint
        for i, (lat, lon) in enumerate(WAYPOINTS, start=1):
            if check_for_emergency(master):
                log_event(writer, master, "EMERGENCY_STOP", f"before GN{i}")
                break

            log_event(writer, master, f"GN{i}_START", f"target={lat},{lon}")
            print(f"[INFO] Going to GN{i}: {lat},{lon}")
            goto(master, lat, lon, 0)
            # Wait loiter time while logging periodically
            waited = 0
            while waited < LOITER_TIME:
                if check_for_emergency(master):
                    log_event(writer, master, "EMERGENCY_STOP", f"during GN{i}")
                    raise KeyboardInterrupt("Emergency stop")
                # log periodic position snapshot
                log_event(writer, master, f"GN{i}_LOITER", f"t={waited}s")
                time.sleep(1)
                waited += 1

            log_event(writer, master, f"GN{i}_REACHED")

        # mission complete -> RTL
        log_event(writer, master, "MISSION_COMPLETE", "sending RTL")
        try:
            master.set_mode_apm("RTL")
            log_event(writer, master, "RTL_CMD_SENT")
            print("[INFO] RTL commanded.")
        except Exception as e:
            log_event(writer, master, "RTL_FAIL", str(e))
            print(f"[WARN] RTL failed: {e}")

        # give some time before exit
        time.sleep(10)

    except KeyboardInterrupt:
        print("[INFO] KeyboardInterrupt or emergency -> setting RTL")
        log_event(writer, master, "INTERRUPT_RTL")
        try:
            master.set_mode_apm("RTL")
        except Exception:
            pass

    finally:
        try:
            # Do not force disarm from code unless desired. (safer)
            # master.arducopter_disarm()
            log_event(writer, master, "MISSION_ENDED")
        except Exception:
            pass
        f.close()
        try:
            master.close()
        except Exception:
            pass
        print("[INFO] Mission script finished.")


if __name__ == "__main__":
    main()

