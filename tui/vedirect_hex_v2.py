#!/usr/bin/env python3
"""
VE.Direct HEX Protocol Tool for Victron SmartSolar/BlueSolar MPPT
v2 - Hybrid TEXT+HEX, corrected history and charger data registers.

Live data is read from the TEXT protocol stream (compatible with all models).
Settings and extended data are read/written via HEX protocol.
Register map verified against official Victron VE.Direct Protocol spec Rev 18.

Usage:
    python3 vedirect_hex_v2.py                        # auto-detect port, read all
    python3 vedirect_hex_v2.py /dev/tty.usbserial-XXX # specify port
    python3 vedirect_hex_v2.py --write                # interactive write mode
"""

import serial
import serial.tools.list_ports
import sys
import time
import argparse
from dataclasses import dataclass, field
from typing import Optional

# ─────────────────────────────────────────────
#  Terminal colours
# ─────────────────────────────────────────────
R   = "\033[0m"
B   = "\033[1m"
DIM = "\033[2m"
CYN = "\033[36m"
GRN = "\033[32m"
YLW = "\033[33m"
RED = "\033[31m"

# ─────────────────────────────────────────────
#  TEXT protocol maps
# ─────────────────────────────────────────────
CS_STATES = {
    "0":   "Off",
    "2":   "Fault",
    "3":   "Bulk",
    "4":   "Absorption",
    "5":   "Float",
    "6":   "Storage",
    "7":   "Equalize (manual)",
    "245": "Wake-up",
    "247": "Auto equalize",
    "250": "Blocked",
    "252": "External control",
    "255": "Unavailable",
}

MPPT_STATES = {
    "0": "Off",
    "1": "Voltage/current limited",
    "2": "MPP tracker active",
}

ERR_CODES = {
    "0":  "No error",
    "2":  "Battery voltage too high",
    "17": "Charger temp too high",
    "18": "Charger over-current",
    "19": "Charger current reversed",
    "20": "Bulk time limit exceeded",
    "21": "Current sensor issue",
    "26": "Terminals overheated",
    "28": "Power stage issue",
    "33": "Input voltage too high",
    "34": "Input current too high",
    "38": "Input shutdown (excess bat V)",
    "39": "Input shutdown (bat I during off)",
}

# ─────────────────────────────────────────────
#  HEX Register definitions
#  Verified against VE.Direct Protocol spec Rev 18
# ─────────────────────────────────────────────
@dataclass
class Register:
    address:     int
    name:        str
    description: str
    unit:        str
    scale:       float
    rw:          str
    fmt:         str  = "d"
    size:        int  = 2
    choices:     dict = field(default_factory=dict)

REGISTERS = [
    # Product info
    Register(0x010A, "Serial Number",        "Device serial number",                  "",     1,      "R",  "s",  0),
    Register(0x010B, "Model Name",           "Model name string",                     "",     1,      "R",  "s",  0),
    Register(0x0140, "Capabilities",         "Device capabilities bitmask",           "",     1,      "R",  "h",  4),
    # Charger data (live, from HEX)
    Register(0xEDD5, "Charger Voltage",      "Voltage at battery terminals",          "V",    0.01,   "R",  "d",  2),
    Register(0xEDD7, "Charger Current",      "Battery + load current combined",       "A",    0.1,    "R",  "d",  2),
    Register(0xEDDB, "Internal Temp",        "Charger internal temperature",          "°C",   0.01,   "R",  "d",  2),
    Register(0xEDDA, "Charger Error",        "Charger error code",                    "",     1,      "R",  "d",  1),
    Register(0xEDD4, "Charger State Info",   "Additional charger state info",         "",     1,      "R",  "h",  1),
    # History (corrected addresses from spec Rev 18)
    Register(0xEDDD, "System Yield",         "Total system yield (non-resettable)",   "kWh",  0.01,   "R",  "d",  4),
    Register(0xEDDC, "User Yield",           "User yield (resettable)",               "kWh",  0.01,   "R",  "d",  4),
    Register(0xEDD3, "Yield Today",          "Yield today",                           "kWh",  0.01,   "R",  "d",  2),
    Register(0xEDD2, "Max Power Today",      "Maximum power today",                   "W",    1,      "R",  "d",  2),
    Register(0xEDD1, "Yield Yesterday",      "Yield yesterday",                       "kWh",  0.01,   "R",  "d",  2),
    Register(0xEDD0, "Max Pwr Yesterday",    "Maximum power yesterday",               "W",    1,      "R",  "d",  2),
    # Battery settings
    Register(0xEDF0, "Max Charge Current",   "Maximum battery charge current",        "A",    0.1,    "RW", "d",  2),
    Register(0xEDF1, "Battery Type",         "Battery type (0xFF=user defined)",      "",     1,      "RW", "d",  1,
             {0:"User defined", 1:"Gel Victron Long Life",
              2:"Gel Victron Deep Discharge", 3:"Gel Victron Deep Discharge (2)",
              4:"AGM Victron Deep Discharge", 5:"Tubular Plate Traction Gel",
              6:"OPzS", 7:"OPzV", 8:"OPzS VRLA", 255:"User defined (custom)"}),
    Register(0xEDF2, "Temp Compensation",    "Battery temp compensation",             "mV/K", 0.01,   "RW", "d",  2),
    Register(0xEDF4, "Equalise Voltage",     "Equalisation voltage",                  "V",    0.01,   "RW", "d",  2),
    Register(0xEDF6, "Float Voltage",        "Float phase target voltage",            "V",    0.01,   "RW", "d",  2),
    Register(0xEDF7, "Absorption Voltage",   "Absorption phase target voltage",       "V",    0.01,   "RW", "d",  2),
    Register(0xEDEF, "Battery Voltage",      "System battery voltage (operational)",  "V",    1,      "R",  "d",  1),
    Register(0xEDEA, "Battery V Setting",    "Battery voltage setting (0=AUTO)",      "V",    1,      "RW", "d",  1),
    Register(0xEDFB, "Absorption Time",      "Max absorption time",                   "h",    0.01,   "RW", "d",  2),
    Register(0xEDFC, "Bulk Time Limit",      "Maximum bulk charge time",              "h",    0.01,   "RW", "d",  2),
    Register(0xEDFD, "Auto Equalise",        "Auto equalisation interval (0=off)",    "days", 1,      "RW", "d",  1),
    Register(0xEDFE, "Adaptive Mode",        "Adaptive absorption mode",              "",     1,      "RW", "d",  1,
             {0:"Off", 1:"On"}),
    Register(0xED2E, "Re-bulk V Offset",     "Re-bulk voltage offset",                "V",    0.01,   "RW", "d",  2),
    # Load output
    Register(0xEDA8, "Load Mode",            "Load output control mode",              "",     1,      "RW", "d",  1,
             {0:"Always off", 1:"BattLife algorithm", 2:"Conventional alg 1",
              3:"Conventional alg 2", 4:"Always on",
              5:"User defined alg 1", 6:"User defined alg 2"}),
    Register(0xEDA9, "Load Low V Off",       "Load switch-off voltage",               "V",    0.01,   "RW", "d",  2),
    Register(0xEDAA, "Load Low V On",        "Load switch-on voltage",                "V",    0.01,   "RW", "d",  2),
    Register(0xEDAD, "Load Current",         "Load output current",                   "A",    0.1,    "R",  "d",  2),
]

REG_BY_ADDR = {r.address: r for r in REGISTERS}

SETTINGS_SECTIONS = [
    ("Product Info",     [0x010A, 0x010B, 0x0140]),
    ("Charger Data",     [0xEDD5, 0xEDD7, 0xEDAD, 0xEDDB, 0xEDDA, 0xEDD4]),
    ("History",          [0xEDDD, 0xEDDC, 0xEDD3, 0xEDD2, 0xEDD1, 0xEDD0]),
    ("Battery Settings", [0xEDF0, 0xEDF1, 0xEDF7, 0xEDF6, 0xEDF4, 0xEDF2,
                          0xEDEF, 0xEDEA, 0xEDFB, 0xEDFC, 0xEDFD, 0xEDFE, 0xED2E]),
    ("Load Output",      [0xEDA8, 0xEDA9, 0xEDAA]),
]

# ─────────────────────────────────────────────
#  TEXT protocol parser
# ─────────────────────────────────────────────

def read_text_frame(ser: serial.Serial, timeout: float = 15.0) -> Optional[dict]:
    """Read one complete TEXT protocol frame, return as dict."""
    frame = {}
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode('ascii', errors='ignore').strip()
        if not line or line.startswith(':'):
            continue
        if '\t' in line:
            key, _, value = line.partition('\t')
            key = key.strip()
            value = value.strip()
            if key == 'Checksum':
                if frame:
                    return frame
            else:
                frame[key] = value
        elif line == 'Checksum' and frame:
            return frame
    return None

def parse_text_live(frame: dict) -> list:
    """Parse a TEXT frame dict into display rows."""
    rows = []

    # Solar
    if 'VPV' in frame:
        v = int(frame['VPV']) / 1000
        rows.append(("Solar Voltage",    f"{v:.2f}", "V"))
    if 'PPV' in frame:
        rows.append(("Solar Power",      frame['PPV'], "W"))
    if 'IL' in frame:
        a = int(frame['IL']) / 1000
        rows.append(("Solar Current",    f"{a:.3f}", "A"))

    # Battery
    if 'V' in frame:
        v = int(frame['V']) / 1000
        rows.append(("Battery Voltage",  f"{v:.3f}", "V"))
    if 'I' in frame:
        a = int(frame['I']) / 1000
        rows.append(("Battery Current",  f"{a:.3f}", "A"))
    if 'VS' in frame:
        v = int(frame['VS']) / 1000
        rows.append(("Starter Voltage",  f"{v:.3f}", "V"))
    if 'VM' in frame:
        v = int(frame['VM']) / 1000
        rows.append(("Mid-point Voltage",f"{v:.3f}", "V"))

    # State
    if 'CS' in frame:
        cs = frame['CS']
        rows.append(("Charge State",     CS_STATES.get(cs, f"Unknown ({cs})"), ""))
    if 'MPPT' in frame:
        m = frame['MPPT']
        rows.append(("MPPT State",       MPPT_STATES.get(m, f"Unknown ({m})"), ""))
    if 'ERR' in frame:
        e = frame['ERR']
        rows.append(("Error",            ERR_CODES.get(e, f"Error {e}"), ""))

    # Load
    if 'LOAD' in frame:
        rows.append(("Load Output",      frame['LOAD'], ""))

    # History (daily from TEXT)
    if 'H19' in frame:
        kwh = int(frame['H19']) / 100
        rows.append(("Yield Total",      f"{kwh:.2f}", "kWh"))
    if 'H20' in frame:
        kwh = int(frame['H20']) / 100
        rows.append(("Yield Today",      f"{kwh:.2f}", "kWh"))
    if 'H21' in frame:
        rows.append(("Max Power Today",  frame['H21'], "W"))
    if 'H22' in frame:
        kwh = int(frame['H22']) / 100
        rows.append(("Yield Yesterday",  f"{kwh:.2f}", "kWh"))
    if 'H23' in frame:
        rows.append(("Max Pwr Yest.",    frame['H23'], "W"))
    if 'HSDS' in frame:
        rows.append(("Day Sequence No.", frame['HSDS'], ""))

    # Device info
    if 'FW' in frame:
        fw = frame['FW']
        rows.append(("Firmware",         f"{int(fw)/100:.2f}", ""))
    if 'PID' in frame:
        rows.append(("Product ID",       frame['PID'], ""))
    if 'SER#' in frame:
        rows.append(("Serial Number",    frame['SER#'], ""))

    return rows

# ─────────────────────────────────────────────
#  HEX protocol
# ─────────────────────────────────────────────

def checksum(data: bytes) -> int:
    return (0x55 - sum(data)) & 0xFF

def build_get(register: int) -> bytes:
    reg_lo = register & 0xFF
    reg_hi = (register >> 8) & 0xFF
    cs = (0x55 - (0x07 + reg_lo + reg_hi)) & 0xFF
    return f":7{reg_lo:02X}{reg_hi:02X}{cs:02X}\n".encode()

def build_set(register: int, value: int, size: int = 2) -> bytes:
    reg_lo    = register & 0xFF
    reg_hi    = (register >> 8) & 0xFF
    mask      = (1 << (size * 8)) - 1
    val_bytes = (value & mask).to_bytes(size, 'little')
    payload   = bytes([0x08, reg_lo, reg_hi, 0x00]) + val_bytes
    cs        = checksum(payload)
    hex_str   = "".join(f"{b:02X}" for b in payload)
    return f":{hex_str}{cs:02X}\n".encode()

def parse_hex_line(line: str) -> Optional[tuple]:
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
    reg       = raw[0] | (raw[1] << 8)
    flags     = raw[2]
    val_bytes = raw[3:-1]
    return cmd_byte, reg, flags, val_bytes

# ─────────────────────────────────────────────
#  Serial helpers
# ─────────────────────────────────────────────

def find_port() -> Optional[str]:
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if "ve direct" in desc or (p.vid == 0x0403 and p.pid == 0x6001):
            return p.device
        if "usbserial" in p.device.lower():
            return p.device
    return None

def open_port(port: str) -> serial.Serial:
    return serial.Serial(
        port=port, baudrate=115200,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1
    )

def wait_for_boundary(ser: serial.Serial, timeout: float = 12.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline()
        if raw and raw.startswith(b'Checksum'):
            time.sleep(0.3)
            ser.reset_input_buffer()
            return True
    return False

def ping(ser: serial.Serial):
    ser.write(b':154\n')
    time.sleep(0.1)

def read_hex_response(ser: serial.Serial, expected_cmd: int,
                      expected_reg: int, timeout: float = 2.0) -> Optional[tuple]:
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode('ascii', errors='ignore').strip()
        if not line.startswith(':'):
            continue
        result = parse_hex_line(line)
        if result is None:
            continue
        cmd, reg, flags, val_bytes = result
        if cmd == expected_cmd and reg == expected_reg:
            return result
    return None

def send_get(ser: serial.Serial, register: int, retries: int = 3) -> Optional[bytes]:
    for _ in range(retries):
        ser.reset_input_buffer()
        ser.write(build_get(register))
        result = read_hex_response(ser, 0x07, register)
        if result is not None:
            cmd, reg, flags, val_bytes = result
            if flags & 0x01:
                return None
            return val_bytes
    return None

def send_set(ser: serial.Serial, register: int,
             raw_value: int, size: int = 2) -> bool:
    ser.reset_input_buffer()
    ser.write(build_set(register, raw_value, size))
    result = read_hex_response(ser, 0x08, register, timeout=2.0)
    if result is None:
        return False
    cmd, reg, flags, val_bytes = result
    if flags == 0x02:
        print(f"  {RED}Not supported (read-only){R}")
    elif flags == 0x04:
        print(f"  {RED}Parameter error (out of range){R}")
    return flags == 0x00

def nvm_save(ser: serial.Serial) -> bool:
    return send_set(ser, 0xEB99, 1, size=1)

# ─────────────────────────────────────────────
#  Value formatting
# ─────────────────────────────────────────────

def decode_value(reg: Register, val_bytes: bytes) -> str:
    if not val_bytes:
        return f"{RED}no data{R}"
    if reg.fmt == "s":
        try:
            return val_bytes.rstrip(b'\x00').decode('ascii')
        except Exception:
            return val_bytes.hex()
    try:
        raw = int.from_bytes(val_bytes, 'little', signed=False)
    except Exception:
        return val_bytes.hex()
    if reg.fmt == "h":
        mask = (1 << (len(val_bytes) * 8)) - 1
        return f"0x{raw & mask:08X}"
    if reg.choices:
        label = reg.choices.get(raw, f"unknown ({raw})")
        return f"{raw} — {label}"
    scaled = raw * reg.scale
    unit   = f" {reg.unit}" if reg.unit else ""
    if reg.scale < 0.01:
        return f"{scaled:.3f}{unit}"
    elif reg.scale < 1:
        return f"{scaled:.2f}{unit}"
    else:
        return f"{scaled:.0f}{unit}"

def scaled_to_raw(reg: Register, value: float) -> int:
    return round(value / reg.scale)

# ─────────────────────────────────────────────
#  Display helpers
# ─────────────────────────────────────────────

def print_header(port: str):
    print(f"\n{B}{CYN}{'─'*64}{R}")
    print(f"{B}{CYN}  Victron MPPT  ·  VE.Direct HEX Tool v2  ·  {port}{R}")
    print(f"{B}{CYN}{'─'*64}{R}\n")

def print_section(title: str):
    print(f"  {B}{YLW}▸ {title}{R}")
    print(f"  {DIM}{'─'*60}{R}")

def print_live_row(label: str, value: str, unit: str):
    unit_str = f" {unit}" if unit else ""
    print(f"  {DIM} R {R}  {DIM}      {R}  {label:<28} {B}{value}{unit_str}{R}")

def print_hex_row(reg: Register, value_str: str):
    rw_badge = f"{GRN}R/W{R}" if "W" in reg.rw else f"{DIM} R {R}"
    addr_str = f"{DIM}0x{reg.address:04X}{R}"
    print(f"  {rw_badge}  {addr_str}  {reg.name:<28} {B}{value_str}{R}")

# ─────────────────────────────────────────────
#  Read all
# ─────────────────────────────────────────────

def read_all(ser: serial.Serial, port: str):
    print_header(port)

    # ── Live data from TEXT protocol ──────────
    print_section("Live Data  (TEXT protocol)")
    print(f"  {DIM}Reading TEXT frame…{R}", end=" ", flush=True)
    frame = read_text_frame(ser)
    if not frame:
        print(f"{RED}Timed out.{R}\n")
    else:
        print(f"{GRN}OK{R}\n")
        rows = parse_text_live(frame)
        for label, value, unit in rows:
            print_live_row(label, value, unit)
    print()

    # ── Extended data + settings from HEX ────
    print(f"  {DIM}Pinging controller for HEX mode…{R}", end=" ", flush=True)
    wait_for_boundary(ser)
    ping(ser)
    print(f"{GRN}OK{R}\n")

    for section_name, addresses in SETTINGS_SECTIONS:
        print_section(section_name)
        ping(ser)
        any_printed = False
        for addr in addresses:
            reg = REG_BY_ADDR.get(addr)
            if not reg:
                continue
            val_bytes = send_get(ser, addr)
            if val_bytes is None:
                continue
            value_str = decode_value(reg, val_bytes)
            print_hex_row(reg, value_str)
            any_printed = True
        if not any_printed:
            print(f"  {DIM}  (no registers returned data){R}")
        print()

# ─────────────────────────────────────────────
#  Interactive write mode
# ─────────────────────────────────────────────

def write_mode(ser: serial.Serial, port: str):
    print_header(port)
    print(f"{B}  Interactive Write Mode{R}\n")
    print(f"  {YLW}Note: To change voltage/temp settings, Battery Type must{R}")
    print(f"  {YLW}be set to User Defined (0xFF = 255) first.{R}\n")

    print(f"  {DIM}Waiting for frame boundary…{R}", end=" ", flush=True)
    if not wait_for_boundary(ser):
        print(f"{RED}Timed out. Check connection.{R}")
        return
    print(f"{GRN}OK{R}")
    print(f"  {DIM}Pinging controller…{R}", end=" ", flush=True)
    ping(ser)
    print(f"{GRN}OK{R}\n")

    writable = [r for r in REGISTERS if "W" in r.rw]
    for i, reg in enumerate(writable):
        print(f"  {DIM}{i+1:>2}.{R}  {CYN}0x{reg.address:04X}{R}  "
              f"{reg.name:<28}  {DIM}{reg.description}{R}")
    print()

    while True:
        try:
            choice = input(f"  {B}Select register number (or 'q' to quit): {R}").strip()
        except (KeyboardInterrupt, EOFError):
            print(); break
        if choice.lower() == 'q':
            break
        try:
            reg = writable[int(choice) - 1]
        except (ValueError, IndexError):
            print(f"  {RED}Invalid selection.{R}\n"); continue

        print(f"\n  {B}{reg.name}{R}  —  {reg.description}")
        if reg.unit:
            print(f"  Unit: {CYN}{reg.unit}{R}   Scale factor: {reg.scale}")
        if reg.choices:
            print(f"  Valid values:")
            for k, v in reg.choices.items():
                print(f"    {k:>4}  =  {v}")

        ping(ser)
        val_bytes = send_get(ser, reg.address)
        if val_bytes is not None:
            print(f"  Current: {B}{decode_value(reg, val_bytes)}{R}")
        else:
            print(f"  {YLW}Could not read current value.{R}")

        try:
            inp = input(f"\n  New value in {reg.unit or 'raw units'} (or 'c' to cancel): ").strip()
        except (KeyboardInterrupt, EOFError):
            print(); break
        if inp.lower() == 'c':
            print(); continue
        try:
            new_value = float(inp)
        except ValueError:
            print(f"  {RED}Invalid value.{R}\n"); continue

        raw_value = scaled_to_raw(reg, new_value)
        print(f"  Writing {new_value} {reg.unit}  →  raw {raw_value} (0x{raw_value & 0xFFFF:04X}) …",
              end=" ", flush=True)
        ping(ser)
        ok = send_set(ser, reg.address, raw_value, size=reg.size)
        if ok:
            print(f"{GRN}OK{R}")
            try:
                if input(f"  Save to NVM? (y/n): ").strip().lower() == 'y':
                    print(f"  Saving …", end=" ", flush=True)
                    ping(ser)
                    print(f"{GRN}OK{R}" if nvm_save(ser) else f"{RED}Failed{R}")
            except (KeyboardInterrupt, EOFError):
                print(); break
        else:
            print(f"{RED}Failed{R}")
        print()

# ─────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="VE.Direct HEX Tool v2 — read/write Victron MPPT settings")
    parser.add_argument("port",    nargs="?",
                        help="Serial port (auto-detected if omitted)")
    parser.add_argument("--write", "-w", action="store_true",
                        help="Interactive write mode")
    args = parser.parse_args()

    port = args.port or find_port()
    if not port:
        print(f"{RED}No VE.Direct device found. Specify port manually:{R}")
        print(f"  python3 vedirect_hex_v2.py /dev/tty.usbserial-XXXXX")
        sys.exit(1)

    print(f"\n{DIM}Connecting to {port} …{R}", end=" ", flush=True)
    try:
        ser = open_port(port)
    except serial.SerialException as e:
        print(f"{RED}Failed{R}\n  {e}"); sys.exit(1)
    print(f"{GRN}OK{R}")

    try:
        if args.write:
            write_mode(ser, port)
        else:
            read_all(ser, port)
    except KeyboardInterrupt:
        print(f"\n\n{DIM}Interrupted.{R}\n")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
