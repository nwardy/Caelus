import serial
import time

port = "COM3"      # change if needed
baud = 115200

def build_msp(cmd, payload):
    size = len(payload)
    frame = bytearray(b"$M<")
    frame.append(size)
    frame.append(cmd)

    checksum = size ^ cmd
    for b in payload:
        checksum ^= b

    frame += payload
    frame.append(checksum)
    return frame

ser = serial.Serial(port, baud, timeout=1)
time.sleep(0.5)

# Motor values (little endian)
# Motor1 = 1000
# Motor2 = 1000
# Motor3 = 1200  <-- spin this one
# Motor4 = 1000

motors = [
    1200,
    1200,
    1200,  # motor 3
    1200
]

payload = bytearray()
for m in motors:
    payload += m.to_bytes(2, 'little')

frame = build_msp(214, payload)

print("Spinning Motor 3...")
ser.write(frame)

time.sleep(3)

# Stop all motors
motors = [1000,1000,1000,1000]
payload = bytearray()
for m in motors:
    payload += m.to_bytes(2, 'little')

frame = build_msp(214, payload)
ser.write(frame)

print("Stopped.")
ser.close()