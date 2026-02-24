# Save as msp_status.py and run: python msp_status.py
# Requires: pip install pyserial

import serial
import time
import sys
from serial.tools import list_ports
import struct

# --- CONFIGURE THIS ---
port = "COM3"         # e.g. 'COM3' on Windows or '/dev/ttyACM0' on Linux
baud = 115200         # try 115200 or 250000 if your FC uses that
timeout = 1.0
# ----------------------

def list_ports_print():
    print("Available serial ports:")
    for p in list_ports.comports():
        print(" ", p.device, "-", p.description)

def build_msp_request(cmd):
    # MSP v1 request frame: $M< size cmd [payload] checksum
    size = 0
    hdr = bytearray(b"$M<")
    frame = hdr + bytearray([size, cmd])
    chksum = size ^ cmd
    frame.append(chksum)
    return frame

def read_frame(ser):
    """
    Read until we find an MSP response header ($M>) then parse it.
    Returns (cmd, payload_bytes) or raises ValueError on checksum/malformed.
    """
    # read bytes until $ found then try to parse header
    start = time.time()
    while True:
        b = ser.read(1)
        if not b:
            raise TimeoutError("Timeout waiting for start of MSP response")
        if b == b'$':
            # peek next two bytes
            next2 = ser.read(2)
            if len(next2) < 2:
                raise TimeoutError("Incomplete header after $")
            if next2 == b'M>':
                break
            # otherwise continue searching (some bytes consumed)
    # read size and cmd
    hdr = ser.read(2)
    if len(hdr) < 2:
        raise TimeoutError("Incomplete header fields")
    size = hdr[0]
    cmd = hdr[1]
    payload = ser.read(size)
    if len(payload) < size:
        raise TimeoutError(f"Expected {size} payload bytes, got {len(payload)}")
    chksum_b = ser.read(1)
    if not chksum_b:
        raise TimeoutError("Missing checksum byte")
    recv_chksum = chksum_b[0]
    calc = size ^ cmd
    for bb in payload:
        calc ^= bb
    if calc != recv_chksum:
        raise ValueError(f"Checksum mismatch: calc=0x{calc:02X} recv=0x{recv_chksum:02X}")
    return cmd, payload

def hexdump(bts):
    return " ".join(f"{x:02X}" for x in bts)

def little_uint(bts):
    """Return little-endian unsigned ints for common widths when possible."""
    out = {}
    ln = len(bts)
    if ln >= 2:
        out['u16_0'] = struct.unpack_from("<H", bts, 0)[0]
    if ln >= 4:
        out['u16_1'] = struct.unpack_from("<H", bts, 2)[0]
    if ln >= 4:
        out['u32_0'] = struct.unpack_from("<I", bts, 0)[0]
    if ln >= 6:
        out['u16_2'] = struct.unpack_from("<H", bts, 4)[0]
    if ln >= 8:
        out['u32_1'] = struct.unpack_from("<I", bts, 4)[0]
    return out

def main():
    print("MSP_STATUS requester")
    list_ports_print()
    print(f"\nOpening port {port} @ {baud}...\n")
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
    except Exception as e:
        print("Error opening port:", e)
        sys.exit(1)

    time.sleep(0.1)   # short settle

    request = build_msp_request(101)  # MSP_STATUS = 101
    print("Sending request:", hexdump(request))
    ser.write(request)
    time.sleep(0.05)

    try:
        cmd, payload = read_frame(ser)
    except Exception as e:
        print("Error reading MSP response:", e)
        ser.close()
        sys.exit(1)

    print("\nReceived MSP response:")
    print(" Command ID:", cmd)
    print(" Payload length:", len(payload))
    print(" Raw payload bytes:", payload)
    print(" Hex dump:", hexdump(payload))

    # Interpret payload as little-endian ints where sensible:
    ints = little_uint(payload)
    if ints:
        print("\nInterpreted little-endian integers (best-effort):")
        for k, v in ints.items():
            print(f"  {k}: {v}")

    # Print bit breakdown of each byte for quick flag inspection
    print("\nPer-byte bit dumps (LSB on right):")
    for i, b in enumerate(payload):
        print(f" byte[{i}] = 0x{b:02X}  bits: {b:08b}")

    ser.close()
    print("\nDone. Paste the above payload (hex) here and I'll decode the fields for you.")

if __name__ == "__main__":
    main()