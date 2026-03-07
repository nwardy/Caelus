# telemetry_reader.py
import threading
import time
import queue

MSP_RC = 105

class TelemetryReader(threading.Thread):
    """
    Continuously reads from serial, reassembles MSP packets and pushes them to an output queue.
    """

    def __init__(self, ser, out_q, read_sleep=0.005, q_size=500):
        super().__init__(daemon=True)
        self.ser = ser
        self.out_q = out_q
        self.read_sleep = read_sleep
        self._buf = bytearray()
        self._running = threading.Event()
        self._running.set()

    def stop(self):
        self._running.clear()

    @staticmethod
    def _parse_rc_payload(payload):
        vals = []
        for i in range(0, len(payload), 2):
            if i + 1 < len(payload):
                import struct
                vals.append(struct.unpack_from("<H", payload, i)[0])
        return vals

    def run(self):
        while self._running.is_set():
            try:
                n = self.ser.in_waiting
            except Exception:
                n = 0
            if n:
                try:
                    chunk = self.ser.read(n)
                except Exception:
                    chunk = b''
                if chunk:
                    self._buf.extend(chunk)
            else:
                try:
                    chunk = self.ser.read(1)
                except Exception:
                    chunk = b''
                if chunk:
                    self._buf.extend(chunk)

            while True:
                start = -1
                for i in range(len(self._buf) - 2):
                    if self._buf[i] == 0x24 and self._buf[i+1] == 0x4D and (self._buf[i+2] == 0x3E or self._buf[i+2] == 0x21):
                        start = i
                        break
                if start == -1:
                    if len(self._buf) > 3:
                        self._buf = self._buf[-3:]
                    break
                if len(self._buf) < start + 6:
                    break
                size = self._buf[start + 3]
                total = start + 3 + 1 + 1 + size + 1
                if len(self._buf) < total:
                    break
                pkt = bytes(self._buf[start:total])
                self._buf = self._buf[total:]
                cmd = pkt[4]
                payload = pkt[5:5+size]
                checksum = pkt[5+size]
                calc = size ^ cmd
                for b in payload:
                    calc ^= b
                if (calc & 0xFF) != checksum:
                    continue
                parsed = None
                if cmd == MSP_RC:
                    parsed = self._parse_rc_payload(payload)
                try:
                    self.out_q.put_nowait((cmd, payload, parsed))
                except queue.Full:
                    pass

            time.sleep(self.read_sleep)