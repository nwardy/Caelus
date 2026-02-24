# quick_msp_status_decode.py
# Paste the payload bytes (raw) into `data` below and run.
data = bytes.fromhex("7C000000230000000000002C000000001D04000000350004")

def le_u16(b, i): return b[i] | (b[i+1]<<8)
def le_u32(b, i): return b[i] | (b[i+1]<<8) | (b[i+2]<<16) | (b[i+3]<<24)

cycle_time = le_u16(data,0)
i2c_errors = le_u16(data,2)
sensors = le_u32(data,4)
# example: status flags / active boxes often live later in the payload
status_flags = le_u32(data,16)
# some firmwares also add profile/system load near the end; show last 4 bytes
tail = data[20:]

print("Cycle time:", cycle_time)
print("I2C errors:", i2c_errors)
print("Sensors bitmask: 0x%08X  (dec %d)" % (sensors, sensors))
print("  sensors bits (common):")
sensor_names = [
    ("ACC", 1<<0),
    ("BARO", 1<<1),
    ("MAG", 1<<2),
    ("GPS", 1<<3),
    ("SONAR", 1<<4)
]
for name,mask in sensor_names:
    print("   %-6s : %s" % (name, bool(sensors & mask)))
print("Raw sensors bits (binary):", format(sensors, '#010b'))
print()
print("Status / box flags (u32 @ bytes 16..19): 0x%08X (dec %d)" % (status_flags, status_flags))
print("  Binary:", format(status_flags, '#034b'))
print()
print("Tail bytes (possible profile/load etc):", tail.hex())