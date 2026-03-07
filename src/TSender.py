#!/usr/bin/env python3
"""
rc_sender.py

RCSender: sends MSP RC frames repeatedly in a background thread.
Includes:
 - channel mapping (1..18)
 - set_channel / set_channels
 - enable/disable transmit loop
 - arm/disarm helper (uses arm channel threshold)
 - ascend(delta_m, duration_s, blocking) simple thrust-to-weight based hop

Tune these params for your craft: MASS_KG, MAX_TOTAL_THRUST_N, MIN_THROTTLE, MAX_SAFE_THROTTLE.
"""

import time
import threading
import struct
import sys

# If you use pyserial, import serial here. Commander will pass serial.Serial instance.
# import serial

# MSP command ids (from common MSP-based firmwares)
MSP_SET_RAW_RC = 200
MSP_RC = 105

class RCSender:
    def __init__(self, ser,
                 lock,
                 num_channels=18,
                 send_channels=16,
                 rate_hz=30,
                 throttle_ch=3,
                 arm_ch=5,
                 min_throttle=1000,
                 max_safe_throttle=1900,
                 neutral=1500,
                 mass_kg=1.5,
                 max_total_thrust_n=30.0):
        """
        ser: serial.Serial instance
        lock: threading.Lock to guard serial access
        """
        self.ser = ser
        self.lock = lock

        # configuration
        self.NUM_CHANNELS = int(num_channels)
        self.SEND_CHANNELS = int(send_channels)
        self.RATE_HZ = float(rate_hz)
        self.THROTTLE_CH = int(throttle_ch)     # channel number (1-indexed)
        self.ARM_CH = int(arm_ch)               # channel number (1-indexed)
        self.MIN_THROTTLE = int(min_throttle)
        self.MAX_SAFE_THROTTLE = int(max_safe_throttle)
        self.NEUTRAL = int(neutral)

        # flight-model params (tune for your craft)
        self.mass_kg = float(mass_kg)
        self.max_total_thrust_n = float(max_total_thrust_n)

        # state
        # channels are 1..NUM_CHANNELS, but store as 0-indexed list
        self.channels = [self.NEUTRAL] * self.NUM_CHANNELS
        self.channels[self.THROTTLE_CH - 1] = self.MIN_THROTTLE
        self.enabled = False   # transmit loop enabled
        self.running = False   # background thread running flag
        self.last_update = time.time()

        # thread & locks
        self._tx_thread = None
        self._op_lock = threading.Lock()   # used by ascend and other operations

    # -------------------------
    # Low-level MSP helpers
    # -------------------------
    def _build_msp(self, cmd, payload=b''):
        # frame $M< size cmd payload checksum
        frame = bytearray(b"$M<")
        size = len(payload)
        frame.append(size & 0xFF)
        frame.append(cmd & 0xFF)
        checksum = size ^ cmd
        for b in payload:
            checksum ^= b
            frame.append(b & 0xFF)
        frame.append(checksum & 0xFF)
        return bytes(frame)

    def _pack_rc_channels(self):
        """
        Pack first SEND_CHANNELS channels as little-endian uint16
        Returns payload bytes suitable for MSP_SET_RAW_RC
        """
        payload = bytearray()
        for i in range(self.SEND_CHANNELS):
            val = int(self.channels[i]) if i < len(self.channels) else self.NEUTRAL
            # enforce safe bounds
            if i == (self.THROTTLE_CH - 1):
                val = max(self.MIN_THROTTLE, min(self.MAX_SAFE_THROTTLE, val))
            else:
                val = max(1000, min(2000, val))
            payload += struct.pack('<H', int(val))
        return bytes(payload)

    # -------------------------
    # Transmit loop
    # -------------------------
    def _tx_loop(self):
        self.running = True
        interval = 1.0 / max(1.0, self.RATE_HZ)
        while self.running:
            if self.enabled:
                payload = self._pack_rc_channels()
                frame = self._build_msp(MSP_SET_RAW_RC, payload)
                try:
                    with self.lock:
                        self.ser.write(frame)
                except Exception as e:
                    # serial errors shouldn't kill the thread
                    print(f"[RCSender] serial write error: {e}", file=sys.stderr)
            time.sleep(interval)
        # thread exiting
        self.running = False

    def start(self):
        if self._tx_thread is None or not self._tx_thread.is_alive():
            self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
            self._tx_thread.start()
            # give it a moment
            time.sleep(0.01)

    def stop(self):
        self.running = False
        if self._tx_thread is not None:
            self._tx_thread.join(timeout=1.0)

    # -------------------------
    # Control API
    # -------------------------
    def enable(self):
        self.enabled = True
        print("[RCSender] enabled sending RC frames")

    def disable(self):
        self.enabled = False
        print("[RCSender] disabled sending RC frames")

    def set_channel(self, ch, value):
        i = int(ch) - 1
        if i < 0 or i >= self.NUM_CHANNELS:
            print(f"[RCSender] set_channel: index {ch} out of range")
            return
        # clamp
        if i == (self.THROTTLE_CH - 1):
            v = max(self.MIN_THROTTLE, min(self.MAX_SAFE_THROTTLE, int(value)))
        else:
            v = max(1000, min(2000, int(value)))
        self.channels[i] = v
        self.last_update = time.time()

    def set_channels(self, values):
        for idx, v in enumerate(values):
            if idx >= self.NUM_CHANNELS:
                break
            self.set_channel(idx + 1, v)

    def get_channels(self):
        return list(self.channels)

    # arm/disarm helpers (sets arm channel to high/low)
    def arm(self):
        # typical threshold: >1700 to arm; here we set to 1900
        self.set_channel(self.ARM_CH, 1900)
        print("[RCSender] arm signal set")

    def disarm(self):
        self.set_channel(self.ARM_CH, 1100)
        print("[RCSender] disarm signal set")

    # -------------------------
    # Flight-model helpers
    # -------------------------
    def _thrust_to_throttle(self, required_thrust_n):
        """
        Map required total thrust (N) to an RC throttle value.
        Linear mapping: 0..max_total_thrust_n -> MIN_THROTTLE..MAX_SAFE_THROTTLE
        """
        req = float(required_thrust_n)
        req = max(0.0, req)
        frac = req / max(1e-6, float(self.max_total_thrust_n))
        frac = max(0.0, min(1.0, frac))
        tmin = float(self.MIN_THROTTLE)
        tmax = float(self.MAX_SAFE_THROTTLE)
        val = int(round(tmin + frac * (tmax - tmin)))
        return max(1000, min(2000, val))

    def ascend(self, delta_m, duration_s=2.0, blocking=False):
        """
        Simple ascend: raise by delta_m meters in duration_s seconds.
        Uses kinematics s = 0.5 * a * t^2 -> a = 2*s/t^2
        required_thrust = mass * (g + a)
        Sends computed throttle for duration_s, then restores previous throttle.
        """
        if delta_m <= 0:
            print("[RCSender] ascend: delta_m must be > 0")
            return

        if not self.enabled:
            print("[RCSender] ascend: sender not enabled. Call enable() first.")
            return

        # compute acceleration
        g = 9.80665
        a = 2.0 * float(delta_m) / max(1e-6, (float(duration_s) ** 2))
        required_thrust = self.mass_kg * (g + a)
        target_throttle = self._thrust_to_throttle(required_thrust)

        def _run_ascend():
            with self._op_lock:
                prev = self.channels[self.THROTTLE_CH - 1]
                print(f"[RCSender] Ascend: {delta_m} m over {duration_s}s -> a={a:.2f} m/s², "
                      f"thrust={required_thrust:.2f} N -> throttle {target_throttle}")
                # set throttle and hold
                self.set_channel(self.THROTTLE_CH, target_throttle)
                start = time.time()
                # keep feeding timestamp to avoid safety timeouts elsewhere
                while time.time() - start < duration_s:
                    self.last_update = time.time()
                    time.sleep(0.02)
                # restore previous throttle
                self.set_channel(self.THROTTLE_CH, prev)
                print("[RCSender] Ascend complete, throttle restored.")

        if blocking:
            _run_ascend()
        else:
            threading.Thread(target=_run_ascend, daemon=True).start()