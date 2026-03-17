#!/usr/bin/env python3
"""
Write test — tests all writable registers with 1ms inter-character delay.
Reads current value and writes it back (safe no-op).
"""

import serial
import time

PORT       = '/dev/cu.usbserial-A40082jb'
BAUD       = 115200
CHAR_DELAY = 0.001   # 1ms between chars — avoids Bus Pirate FIFO overflow

TEST_REGS = [
    (0xEDF0, "Max Charge Current",  2),
    (0xEDF2, "Temp Compensation",   2),
    (0xEDF4, "Equalise Voltage",    2),
    (0xEDF6, "Float Voltage",       2),
    (0xEDF7, "Absorption Voltage",  2),
    (0xEDEA, "Battery V Setting",   1),
    (0xEDFB, "Absorption Time",     2),
    (0xEDFC, "Bulk Time Limit",     2),
    (0xEDFD, "Auto Equalise",       1),
    (0xEDFE, "Adaptive Mode",       1),
    (0xED2E, "Re-bulk V Offset",    2),
]

def build_get(register):
    reg_lo = register & 0xFF
    reg_hi = (register >> 8) & 0xFF
    cs = (0x55 - (0x07 + reg_lo + reg_hi)) & 0xFF
    return f":7{reg_lo:02X}{reg_hi:02X}{cs:02X}\n".encode()

def build_set(register, value, size=2):
    reg_lo    = register & 0xFF
    reg_hi    = (register >> 8) & 0xFF
    mask      = (1 << (size * 8)) - 1
    val_bytes = (value & mask).to_bytes(size, 'little')
    data      = bytes([reg_lo, reg_hi, 0x00]) + val_bytes
    cs        = (0x55 - (0x08 + sum(data))) & 0xFF
    hex_str   = "".join(f"{b:02X}" for b in data)
    return f":8{hex_str}{cs:02X}\n".encode()

def write_slow(ser, frame):
    for byte in frame:
        ser.write(bytes([byte]))
        time.sleep(CHAR_DELAY)

def parse_hex(line):
    line = line.strip()
    if not line.startswith(':') or len(line) < 4:
        return None
    cmd_nibble = line[1]
    rest = line[2:]
    if len(rest) % 2 != 0:
        return None
    try:
        raw = bytes.fromhex(rest)
    except ValueError:
        return None
    cmd_byte = int(cmd_nibble, 16)
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

def do_get(ser, register, timeout=3.0):
    ping(ser)
    ser.reset_input_buffer()
    write_slow(ser, build_get(register))
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline()
        if not raw: continue
        line = raw.decode('ascii', errors='ignore').strip()
        if not line.startswith(':'): continue
        r = parse_hex(line)
        if r and r[0] == 0x07 and r[1] == register:
            return r
    return None

def do_set(ser, register, value, size):
    wait_boundary(ser)
    ser.reset_input_buffer()
    frame = build_set(register, value, size)
    write_slow(ser, frame)
    # wait for SET response (cmd 8) or error (cmd 4), ignore async
    deadline = time.time() + 4.0
    while time.time() < deadline:
        raw = ser.readline()
        if not raw: continue
        line = raw.decode('ascii', errors='ignore').strip()
        if not line.startswith(':'): continue
        r = parse_hex(line)
        if r and r[0] in (0x08, 0x04):
            return r
    return None

def decode_set_response(r):
    if r is None:
        return "\033[31mno response\033[0m"
    cmd, reg, flags, val_bytes = r
    if cmd == 0x04:
        return "\033[31mFrame error\033[0m"
    if flags == 0x00: return "\033[32mOK\033[0m"
    if flags == 0x01: return "\033[31mUnknown ID\033[0m"
    if flags == 0x02: return "\033[31mRead-only\033[0m"
    if flags == 0x04: return "\033[31mParam error\033[0m"
    return f"\033[31mflags=0x{flags:02X}\033[0m"

# ── main ──────────────────────────────────────────────────────────────

print(f"\nConnecting to {PORT}...")
ser = serial.Serial(PORT, BAUD, timeout=0.5)

print("Waiting for frame boundary...")
if not wait_boundary(ser):
    print("ERROR: timed out"); ser.close(); exit(1)

print(f"\n{'Register':<26} {'Current':>10}  {'SET result':>12}")
print("─" * 55)

for addr, name, size in TEST_REGS:
    result = do_get(ser, addr)
    if result is None or result[2] & 0x01:
        print(f"  {name:<24} {'no GET':>10}  {'—':>12}")
        continue

    cmd, reg, flags, val_bytes = result
    current = int.from_bytes(val_bytes, 'little') if val_bytes else 0
    current_str = f"{current}({len(val_bytes)}B)"

    set_result = do_set(ser, addr, current, size)
    print(f"  {name:<24} {current_str:>10}  {decode_set_response(set_result):>12}")

ser.close()
print("\nDone.")
