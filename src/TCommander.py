#!/usr/bin/env python3
"""
commander.py

Simple CLI wrapper around RCSender. Starts serial, instantiates RCSender,
and exposes a small set of commands:

Commands:
  start             - start transmit thread
  enable            - enable sending RC frames
  disable           - disable sending RC frames
  arm               - set arm channel high (arm)
  disarm            - set arm channel low (disarm)
  set <ch> <val>    - set channel number to value (1-indexed)
  show              - print current channels
  map throttle <ch> - change throttle channel mapping (1-indexed)
  map arm <ch>      - change arm channel mapping (1-indexed)
  go up <meters>    - ascend by meters (e.g. "go up 1", "up 0.5")
  quit / exit       - exit
"""

import serial
import threading
import time
import sys
import re

from TSender import RCSender
from TReciver import TelemetryReader


DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 115200

def open_serial(port, baud, timeout=0.1):
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(0.05)
        return ser
    except Exception as e:
        print(f"[commander] failed to open serial {port}@{baud}: {e}")
        return None

def repl_loop(rc):
    print("Simple commander. Type 'help' for commands.")
    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break
        if not cmd:
            continue
        lc = cmd.lower().strip()

        if lc in ("quit", "exit"):
            break

        if lc == "help":
            print("Commands: start, enable, disable, arm, disarm, set <ch> <val>, show, map throttle <ch>, map arm <ch>, go up <meters>, quit")
            continue

        if lc == "start":
            rc.start()
            print("[commander] transmit thread started")
            continue

        if lc == "enable":
            rc.enable()
            continue

        if lc == "disable":
            rc.disable()
            continue

        if lc == "arm":
            rc.arm()
            continue

        if lc == "disarm":
            rc.disarm()
            continue

        # set <ch> <val>
        m = re.match(r'^set\s+(\d+)\s+(\d+)', lc)
        if m:
            ch = int(m.group(1))
            val = int(m.group(2))
            rc.set_channel(ch, val)
            print(f"[commander] set ch{ch} -> {val}")
            continue

        if lc == "show":
            chans = rc.get_channels()
            print("Channels:")
            for i, v in enumerate(chans, start=1):
                mark = ""
                if i == rc.THROTTLE_CH:
                    mark = "(throttle)"
                if i == rc.ARM_CH:
                    mark = "(arm)"
                print(f"  {i:02d}: {v} {mark}")
            continue

        # map throttle <ch>
        m = re.match(r'^map\s+throttle\s+(\d+)', lc)
        if m:
            newch = int(m.group(1))
            if 1 <= newch <= rc.NUM_CHANNELS:
                rc.THROTTLE_CH = newch
                print(f"[commander] throttle channel mapped to {newch}")
            else:
                print("invalid channel")
            continue

        # map arm <ch>
        m = re.match(r'^map\s+arm\s+(\d+)', lc)
        if m:
            newch = int(m.group(1))
            if 1 <= newch <= rc.NUM_CHANNELS:
                rc.ARM_CH = newch
                print(f"[commander] arm channel mapped to {newch}")
            else:
                print("invalid channel")
            continue

        # go up <meters> or up <m>
        if lc.startswith("go up ") or lc.startswith("up "):
            m = re.search(r"([0-9]*\.?[0-9]+)", cmd)
            if m:
                dz = float(m.group(1))
                # conservative default: 2 seconds per meter (tweakable)
                duration = 2.0 * max(0.25, dz)
                print(f"[commander] requesting ascend {dz} m over {duration:.2f}s")
                rc.ascend(dz, duration_s=duration, blocking=False)
            else:
                print("usage: go up <meters> (e.g. 'go up 1' or 'up 0.5')")
            continue

        print("Unknown command. Type 'help' for list.")

def main():
    port = DEFAULT_PORT
    baud = DEFAULT_BAUD
    if len(sys.argv) >= 2:
        port = sys.argv[1]
    if len(sys.argv) >= 3:
        baud = int(sys.argv[2])

    ser = open_serial(port, baud)
    if ser is None:
        print("Unable to open serial port. Exiting.")
        return

    lock = threading.Lock()
    # create RCSender: tune mass_kg and max_total_thrust_n
    rc = RCSender(ser=ser,
                  lock=lock,
                  num_channels=18,
                  send_channels=16,
                  rate_hz=30,
                  throttle_ch=3,
                  arm_ch=5,
                  min_throttle=1000,
                  max_safe_throttle=1900,
                  neutral=1500,
                  mass_kg=1.6,              # <-- set your vehicle mass (kg)
                  max_total_thrust_n=28.0)  # <-- total thrust (N) at full throttle (tweak)

    # start tx loop
    rc.start()

    try:
        repl_loop(rc)
    finally:
        print("[commander] shutting down...")
        rc.disable()
        rc.stop()
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()