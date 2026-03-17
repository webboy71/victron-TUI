import serial
import time

def checksum(data):
    return (0x55 - sum(data)) & 0xFF

def build_set(register, value, size=2):
    reg_lo = register & 0xFF
    reg_hi = (register >> 8) & 0xFF
    mask = (1 << (size * 8)) - 1
    val_bytes = (value & mask).to_bytes(size, 'little')
    payload = bytes([0x08, reg_lo, reg_hi, 0x00]) + val_bytes
    cs = checksum(payload)
    val_hex = "".join(f"{b:02X}" for b in val_bytes)
    return f':8{reg_lo:02X}{reg_hi:02X}00{val_hex}{cs:02X}\n'.encode()

ser = serial.Serial('/dev/cu.usbserial-A40082jb', 115200, timeout=0.1)

# Absorption voltage 0xEDF2, typical value 14.40V = raw 1440
# Writing 1440 back to itself — safe no-op
frame = build_set(0xEDF2, 1440)
print(f"SET frame: {frame}")

print("Waiting for boundary...")
deadline = time.time() + 10
while time.time() < deadline:
    raw = ser.readline()
    if raw.startswith(b'Checksum'):
        time.sleep(0.3)
        ser.reset_input_buffer()
        ser.write(frame)
        print("SET sent")
        break

deadline = time.time() + 4
while time.time() < deadline:
    raw = ser.readline()
    if not raw:
        continue
    line = raw.decode('ascii', errors='ignore').strip()
    if line.startswith(':'):
        print(f"  HEX: {line}")
    else:
        print(f"  TEXT: {line}")

ser.close()