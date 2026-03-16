#!/usr/bin/env python3
"""
Test script: fetch 30-day history from registers 0x1050-0x106E.
Layout confirmed: byte[1]=yield×10Wh, byte[30]=max power W.
No ping between GETs to avoid disrupting burst reads.
"""

import serial
import time

PORT = '/dev/cu.usbserial-A40082jb'
BAUD = 115200
CHAR_DELAY = 0.001

def write_slow(ser, data):
    for byte in data:
        ser.write(bytes([byte]))
        time.sleep(CHAR_DELAY)

def build_get(register):
    reg_lo = register & 0xFF
    reg_hi = (register >> 8) & 0xFF
    cs = (0x55 - (0x07 + reg_lo + reg_hi)) & 0xFF
    return f":7{reg_lo:02X}{reg_hi:02X}{cs:02X}\n".encode()

def parse_hex(line):
    line = line.strip()
    if not line.startswith(':') or len(line) < 4:
        return None
    rest = line[2:]
    if len(rest) % 2 != 0:
        return None
    try:
        raw = bytes.fromhex(rest)
    except ValueError:
        return None
    cmd_byte = int(line[1], 16)
    if (cmd_byte + sum(raw)) & 0xFF != 0x55:
        return None
    if len(raw) < 3:
        return None
    return cmd_byte, raw[0] | (raw[1] << 8), raw[2], raw[3:-1]

def wait_boundary(ser, timeout=12.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline()
        if raw and raw.startswith(b'Checksum'):
            time.sleep(0.3)
            ser.reset_input_buffer()
            return True
    return False

def ping(ser):
    write_slow(ser, b':154\n')
    time.sleep(0.1)

def get_register(ser, addr, timeout=3.0):
    ser.reset_input_buffer()
    write_slow(ser, build_get(addr))
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline()
        if not raw: continue
        line = raw.decode('ascii', errors='ignore').strip()
        if not line.startswith(':'): continue
        r = parse_hex(line)
        if r and r[0] == 0x07 and r[1] == addr:
            return r
    return None

# ── main ──────────────────────────────────────────────────────────────

print(f"\nConnecting to {PORT}...")
ser = serial.Serial(PORT, BAUD, timeout=0.5)

print("Waiting for boundary...")
if not wait_boundary(ser):
    print("ERROR: timed out"); ser.close(); exit(1)

# Single ping after boundary to enter HEX mode
ping(ser)
# Small delay to let async frames settle
time.sleep(0.5)
ser.reset_input_buffer()

print("\nFetching day records...")
print(f"  {'Day':<6} {'Addr':<8} {'Yield':>10} {'MaxPwr':>10} {'Raw bytes'}")
print("  " + "─"*60)

days = []
for i in range(31):
    addr = 0x1050 + i

    # Re-ping every 5 registers to keep HEX mode alive
    if i > 0 and i % 5 == 0:
        ping(ser)
        time.sleep(0.2)
        ser.reset_input_buffer()

    r = get_register(ser, addr)

    if r is None:
        print(f"  {i:<6} 0x{addr:04X}   no response — stopping")
        break

    cmd, reg, flags, val = r
    if flags & 0x01:
        print(f"  {i:<6} 0x{addr:04X}   unknown register — end of data")
        break
    if not val or len(val) < 31:
        print(f"  {i:<6} 0x{addr:04X}   short response ({len(val) if val else 0} bytes)")
        continue

    yield_wh  = val[1] * 10        # byte[1] × 10Wh
    max_pwr   = val[30]             # byte[30] in W
    days.append((i, yield_wh, max_pwr))

    marker = " ← today (incomplete)" if i == 0 else ""
    print(f"  {i:<6} 0x{addr:04X}   {yield_wh:>7}Wh  {max_pwr:>7}W  {val[:4].hex()}...{marker}")

print()
print(f"Total records: {len(days)}")
if days:
    total_wh = sum(d[1] for d in days)
    peak_w   = max(d[2] for d in days)
    print(f"Total yield (excluding today): {total_wh}Wh")
    print(f"Peak power across all days: {peak_w}W")

ser.close()
print("\nDone.")
