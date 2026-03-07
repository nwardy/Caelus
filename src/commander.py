# commander.py
import serial
import threading
import queue
import sys
import time

from rc_sender import RCSender
from telemetry_reader import TelemetryReader

PORT = "COM3"
BAUD = 115200

NUM_CHANNELS = 18
SEND_CHANNELS = 16

GETAUX_TIMEOUT = 0.8

def get_msp_rc_via_queue(ser, write_lock, reader_q, timeout=GETAUX_TIMEOUT):
    try:
        while True:
            reader_q.get_nowait()
    except queue.Empty:
        pass

    with write_lock:
        try:
            frame = bytearray(b"$M<")
            frame.append(0)
            frame.append(105)
            checksum = 0 ^ 105
            frame.append(checksum & 0xFF)
            ser.write(bytes(frame))
            try:
                ser.flush()
            except Exception:
                pass
        except Exception:
            return None

    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            cmd, payload, parsed = reader_q.get(timeout=deadline - time.time())
        except queue.Empty:
            return None
        if cmd == 105:
            return parsed
    return None


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.02)
        print("[SERIAL] Opened", PORT)
    except Exception as e:
        print("Serial error:", e)
        sys.exit(1)

    write_lock = threading.Lock()
    reader_q = queue.Queue(maxsize=500)

    reader = TelemetryReader(ser, reader_q)
    reader.start()

    rc = RCSender(ser, write_lock,
                  num_channels=NUM_CHANNELS,
                  send_channels=SEND_CHANNELS,
                  rate_hz=30,
                  throttle_ch=3,
                  arm_ch=5,
                  min_throttle=800,
                  max_safe_throttle=1900,
                  neutral=1500)
    rc.start()

    print("Commands:")
    print(" enable | disable | set <ch> <value> | status")
    print(" getaux | telemon on | telemon off | map throttle <ch> | map arm <ch>")
    print(" arm | disarm | showmap | quit")
    print(" PROPS OFF while testing.\n")

    tele_event = threading.Event()

    try:
        while True:
            raw = input("> ")
            if raw is None:
                continue
            cmd = raw.strip()
            if not cmd:
                continue
            parts = cmd.split()
            lc = cmd.lower().strip()

            if lc == "enable":
                rc.enable()
                continue
            if lc == "disable":
                rc.disable()
                continue
            if lc.startswith("set "):
                if len(parts) >= 3:
                    try:
                        ch = int(parts[1]); val = int(parts[2])
                        rc.set_channel(ch, val)
                    except Exception:
                        print("usage: set <ch> <value>")
                else:
                    print("usage: set <ch> <value>")
                continue
            if lc == "status":
                print("Enabled:", rc.enabled)
                print("Set Channels (first 8):", rc.channels[:8])
                continue
            if lc in ("getaux", "aux"):
                chans = get_msp_rc_via_queue(ser, write_lock, reader_q, timeout=GETAUX_TIMEOUT)
                if chans is None:
                    print("No MSP_RC reply received.")
                else:
                    for i in range(NUM_CHANNELS):
                        if i < len(chans):
                            print(f"CH{i+1}: {chans[i]}")
                        else:
                            print(f"CH{i+1}: ---")
                continue
            if lc == "telemon on":
                tele_event.set()
                print("Telemetry ON")
                def poller():
                    while tele_event.is_set():
                        chans = get_msp_rc_via_queue(ser, write_lock, reader_q, timeout=GETAUX_TIMEOUT)
                        if chans:
                            print("[TELEM] " + " | ".join([f"CH{i+1}={chans[i]}" if i < len(chans) else f"CH{i+1}=---" for i in range(NUM_CHANNELS)]))
                        time.sleep(1.0)
                threading.Thread(target=poller, daemon=True).start()
                continue
            if lc == "telemon off":
                tele_event.clear(); print("Telemetry OFF"); continue
            if lc == "arm":
                rc.arm(); continue
            if lc == "disarm":
                rc.disarm(); continue
            if lc.startswith("map "):
                if len(parts) == 3 and parts[1].lower() == "throttle":
                    try:
                        newch = int(parts[2])
                        if not 1 <= newch <= NUM_CHANNELS:
                            print("channel must be 1-{}".format(NUM_CHANNELS)); continue
                        rc.THROTTLE_CH = newch
                        rc.set_channel(newch, rc.channels[newch-1])
                        print("Throttle mapped to CH", newch)
                    except Exception:
                        print("usage: map throttle <ch>")
                elif len(parts) == 3 and parts[1].lower() == "arm":
                    try:
                        newch = int(parts[2])
                        if not 1 <= newch <= NUM_CHANNELS:
                            print("channel must be 1-{}".format(NUM_CHANNELS)); continue
                        rc.ARM_CH = newch
                        print("Arm channel set to CH", newch)
                    except Exception:
                        print("usage: map arm <ch>")
                else:
                    print("usage: map throttle <ch>  OR  map arm <ch>")
                continue
            if lc == "showmap":
                print(f"Throttle: CH{rc.THROTTLE_CH}  Arm: CH{rc.ARM_CH}")
                continue
            if lc in ("quit", "exit"):
                break
            print("Unknown command")
    except KeyboardInterrupt:
        pass

    print("Shutting down...")
    tele_event.clear()
    reader.stop()
    rc.stop()
    ser.close()

if __name__ == "__main__":
    main()