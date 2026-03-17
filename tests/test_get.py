import serial, time

PORT = '/dev/cu.usbserial-A40082jb'

def build_get(register):
    reg_lo = register & 0xFF
    reg_hi = (register >> 8) & 0xFF
    cs = (0x55 - (0x07 + reg_lo + reg_hi)) & 0xFF
    return f":7{reg_lo:02X}{reg_hi:02X}{cs:02X}\n".encode()

ser = serial.Serial(PORT, 115200, timeout=0.1)

print("Waiting for boundary...")
deadline = time.time() + 15
while time.time() < deadline:
    raw = ser.readline()
    if raw.startswith(b'Checksum'):
        time.sleep(0.3)
        ser.reset_input_buffer()
        break

ser.write(b':154\n')
time.sleep(0.1)

for name, addr in [("Absorption V", 0xEDF2), ("Float V",    0xEDF4),
                   ("Battery V",    0xED8D), ("Panel V",    0xEDD0),
                   ("Bat Current",  0xEDBB)]:
    ser.reset_input_buffer()
    frame = build_get(addr)
    ser.write(frame)
    deadline = time.time() + 2
    while time.time() < deadline:
        raw = ser.readline()
        if not raw: continue
        line = raw.decode('ascii', errors='ignore').strip()
        if line.startswith(':7'):
            # parse raw bytes and show decimal
            rest = line[2:]
            data = bytes.fromhex(rest)
            val_bytes = data[3:-1]
            raw_int = int.from_bytes(val_bytes, 'little', signed=False)
            print(f"  {name:<14} 0x{addr:04X}  raw={raw_int:6d}  hex={val_bytes.hex()}  frame={line}")
            break

ser.close()