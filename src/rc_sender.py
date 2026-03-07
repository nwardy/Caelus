# rc_sender.py
import struct
import threading
import time

MSP_SET_RAW_RC = 200

class RCSender:
    """
    Responsible for building and sending MSP_SET_RAW_RC frames repeatedly.
    """

    def __init__(self, ser, lock,
                 num_channels=18,
                 send_channels=16,
                 rate_hz=30,
                 throttle_ch=3,
                 arm_ch=5,
                 min_throttle=800,
                 max_safe_throttle=1900,
                 neutral=1500):
        self.ser = ser
        self.lock = lock
        self.NUM_CHANNELS = num_channels
        self.SEND_CHANNELS = send_channels
        self.RATE_HZ = rate_hz

        self.THROTTLE_CH = throttle_ch
        self.ARM_CH = arm_ch
        self.MIN_THROTTLE = min_throttle
        self.MAX_SAFE_THROTTLE = max_safe_throttle
        self.NEUTRAL = neutral

        self.channels = [self.NEUTRAL] * self.NUM_CHANNELS
        if 1 <= self.THROTTLE_CH <= self.NUM_CHANNELS:
            self.channels[self.THROTTLE_CH - 1] = self.MIN_THROTTLE

        self.enabled = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    @staticmethod
    def build_msp(cmd, payload=b''):
        frame = bytearray(b"$M<")
        size = len(payload) & 0xFF
        frame.append(size)
        frame.append(cmd & 0xFF)
        checksum = size ^ (cmd & 0xFF)
        for b in payload:
            checksum ^= b
        frame += payload
        frame.append(checksum & 0xFF)
        return bytes(frame)

    def _pack_channels(self, channels, send_count=None):
        send_count = send_count or self.SEND_CHANNELS
        ch = list(channels)[:send_count]
        for i in range(len(ch)):
            v = ch[i]
            if not isinstance(v, (int, float)) or v is None or v == 0:
                ch[i] = self.NEUTRAL
            else:
                ch[i] = int(max(1000, min(2000, v)))
        while len(ch) < send_count:
            ch.append(self.NEUTRAL)
        return struct.pack("<" + "H" * send_count, *ch)

    def start(self):
        self._stop.clear()
        if not self._thread.is_alive():
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def enable(self):
        self.enabled = True
        self.last_update = time.time()
        print("[RC] ENABLED")

    def disable(self):
        self.enabled = False
        safe = [self.NEUTRAL] * self.NUM_CHANNELS
        if 1 <= self.THROTTLE_CH <= self.NUM_CHANNELS:
            safe[self.THROTTLE_CH - 1] = self.MIN_THROTTLE
        if 1 <= self.ARM_CH <= self.NUM_CHANNELS:
            safe[self.ARM_CH - 1] = 1000
        payload = self._pack_channels(safe, send_count=self.SEND_CHANNELS)
        frame = self.build_msp(MSP_SET_RAW_RC, payload)
        with self.lock:
            for _ in range(4):
                try:
                    self.ser.write(frame)
                except Exception:
                    pass
                time.sleep(0.03)
        print("[RC] DISABLED")

    def emergency_stop(self):
        self.disable()

    def set_channel(self, ch, value):
        if not 1 <= ch <= self.NUM_CHANNELS:
            print("Channel must be 1-{}".format(self.NUM_CHANNELS))
            return
        if ch == self.THROTTLE_CH:
            v = max(self.MIN_THROTTLE, min(self.MAX_SAFE_THROTTLE, int(value)))
        else:
            v = max(1000, min(2000, int(value)))
        self.channels[ch-1] = v
        self.last_update = time.time()
        print(f"[RC] CH{ch} = {self.channels[ch-1]}")

    def arm(self):
        self.set_channel(self.ARM_CH, 1900)
        payload = self._pack_channels(self.channels, send_count=self.SEND_CHANNELS)
        frame = self.build_msp(MSP_SET_RAW_RC, payload)
        with self.lock:
            try:
                self.ser.write(frame)
            except Exception:
                pass
        print("[RC] ARM command sent")

    def disarm(self):
        self.set_channel(self.ARM_CH, 1000)
        payload = self._pack_channels(self.channels, send_count=self.SEND_CHANNELS)
        frame = self.build_msp(MSP_SET_RAW_RC, payload)
        with self.lock:
            try:
                self.ser.write(frame)
            except Exception:
                pass
        print("[RC] DISARM command sent")

    def _loop(self):
        interval = 1.0 / max(1.0, float(self.RATE_HZ))
        while not self._stop.is_set():
            if not self.enabled:
                send = [self.NEUTRAL] * self.NUM_CHANNELS
                if 1 <= self.THROTTLE_CH <= self.NUM_CHANNELS:
                    send[self.THROTTLE_CH - 1] = self.MIN_THROTTLE
                if 1 <= self.ARM_CH <= self.NUM_CHANNELS:
                    send[self.ARM_CH - 1] = 1000
                payload = self._pack_channels(send, send_count=self.SEND_CHANNELS)
            else:
                payload = self._pack_channels(self.channels, send_count=self.SEND_CHANNELS)

            frame = self.build_msp(MSP_SET_RAW_RC, payload)
            with self.lock:
                try:
                    self.ser.write(frame)
                except Exception:
                    pass
            time.sleep(interval)