#!/usr/bin/env python3
"""
VE.Direct HEX Protocol Tool for Victron SmartSolar/BlueSolar MPPT
Reads and writes controller settings via the VE.Direct HEX protocol.

Usage:
    python3 vedirect_hex.py                        # auto-detect port, read all
    python3 vedirect_hex.py /dev/tty.usbserial-XXX # specify port
    python3 vedirect_hex.py --write                # interactive write mode
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
#  Register definitions
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
    choices:     dict = field(default_factory=dict)

REGISTERS = [
    Register(0x0100, "Product ID",           "Product identifier",                    "",     1,     "R",  "h"),
    Register(0x0102, "Firmware Version",     "Application firmware version",          "",     0.01,  "R"),
    Register(0x010A, "Serial Number",        "Device serial number",                  "",     1,     "R",  "s"),
    Register(0xED8D, "Battery Voltage",      "Measured battery voltage",              "V",    0.01,  "R"),
    Register(0xEDBB, "Battery Current",      "Measured battery current",              "A",    0.1,   "R"),
    Register(0xEDD0, "Panel Voltage",        "Measured PV panel voltage",             "V",    0.01,  "R"),
    Register(0xEDBC, "Panel Power",          "Measured PV panel power",               "W",    1,     "R"),
    Register(0xEDAD, "Load Current",         "Load output current",                   "A",    0.1,   "R"),
    Register(0x0201, "Device State",         "Charger operating state",               "",     1,     "R",  "d",
             {0:"Off", 2:"Fault", 3:"Bulk", 4:"Absorption", 5:"Float",
              7:"Equalize (manual)", 245:"Starting up", 247:"Auto equalize",
              252:"External control"}),
    Register(0x0205, "Error Code",           "Active error code",                     "",     1,     "R",  "d",
             {0:"No error", 1:"Battery temp too high", 2:"Battery voltage too high",
              17:"Charger temp too high", 18:"Charger over-current",
              19:"Charger current reversed", 20:"Bulk time limit exceeded",
              21:"Current sensor issue", 26:"Terminals overheated",
              28:"Power stage issue", 33:"Input voltage too high",
              34:"Input current too high", 38:"Input shutdown (excess bat V)",
              39:"Input shutdown (bat current during off)"}),
    Register(0xEDF0, "Max Charge Current",   "Maximum battery charge current",        "A",    0.1,   "RW"),
    Register(0xEDF2, "Absorption Voltage",   "Absorption phase target voltage",       "V",    0.01,  "RW"),
    Register(0xEDF4, "Float Voltage",        "Float phase target voltage",            "V",    0.01,  "RW"),
    Register(0xEDF6, "Equalise Voltage",     "Equalisation voltage",                  "V",    0.01,  "RW"),
    Register(0xEDCA, "Bulk Time Limit",      "Maximum bulk charge time",              "min",  1,     "RW"),
    Register(0xEDCB, "Absorption Time",      "Fixed absorption time",                 "min",  1,     "RW"),
    Register(0xEDCC, "Absorption Repeat",    "Repeat absorption interval",            "days", 1,     "RW"),
    Register(0xEDD3, "Battery Type",         "Battery type preset",                   "",     1,     "RW", "d",
             {1:"Gel Victron Long Life", 2:"Gel Victron Deep Discharge",
              3:"Gel Victron Deep Discharge (2)", 4:"AGM Victron Deep Discharge",
              5:"Tubular Plate Traction Gel", 6:"OPzS", 7:"OPzV",
              8:"OPzS VRLA", 255:"User defined"}),
    Register(0xEDDC, "Battery Capacity",     "Battery capacity",                      "Ah",   1,     "RW"),
    Register(0xEDD7, "Temperature Comp",     "Battery temp compensation coefficient", "mV/K", 1,     "RW"),
    Register(0xEDEF, "Battery Voltage Set",  "System battery voltage setting",        "V",    0.01,  "RW"),
    Register(0xEDD4, "Auto Equalise",        "Automatic equalisation interval",       "days", 1,     "RW"),
    Register(0xEDDE, "Absorption Mode",      "Adaptive vs fixed absorption",          "",     1,     "RW", "d",
             {0:"Adaptive (default)", 1:"Fixed time"}),
    Register(0xEDA8, "Load Mode",            "Load output control mode",              "",     1,     "RW", "d",
             {0:"Always off", 1:"BattLife algorithm", 2:"Conventional alg 1",
              3:"Conventional alg 2", 4:"Always on",
              5:"User defined alg 1", 6:"User defined alg 2"}),
    Register(0xEDA9, "Load Low V Off",       "Load switch-off voltage",               "V",    0.01,  "RW"),
    Register(0xEDAA, "Load Low V On",        "Load switch-on voltage",                "V",    0.01,  "RW"),
    Register(0xEE10, "Yield Total",          "Total yield (resettable)",              "kWh",  0.01,  "R"),
    Register(0xEE11, "Yield Today",          "Yield today",                           "kWh",  0.01,  "R"),
    Register(0xEE12, "Max Power Today",      "Maximum power today",                   "W",    1,     "R"),
    Register(0xEE13, "Yield Yesterday",      "Yield yesterday",                       "kWh",  0.01,  "R"),
    Register(0xEE14, "Max Power Yesterday",  "Maximum power yesterday",               "W",    1,     "R"),
]

REG_BY_ADDR = {r.address: r for r in REGISTERS}

SECTIONS = [
    ("Product Info",    [0x0100, 0x0102, 0x010A]),
    ("Live Data",       [0xED8D, 0xEDBB, 0xEDD0, 0xEDBC, 0xEDAD, 0x0201, 0x0205]),
    ("Charge Settings", [0xEDF0, 0xEDF2, 0xEDF4, 0xEDF6, 0xEDCA, 0xEDCB,
                         0xEDCC, 0xEDD3, 0xEDDC, 0xEDD7, 0xEDEF, 0xEDD4, 0xEDDE]),
    ("Load Output",     [0xEDA8, 0xEDA9, 0xEDAA]),
    ("History",         [0xEE10, 0xEE11, 0xEE12, 0xEE13, 0xEE14]),
]

# ─────────────────────────────────────────────
#  HEX protocol
# ─────────────────────────────────────────────

def checksum(data: bytes) -> int:
    return (0x55 - sum(data)) & 0xFF

def build_get(register: int) -> bytes:
    reg_lo  = register & 0xFF
    reg_hi  = (register >> 8) & 0xFF
    payload = bytes([0x07, reg_lo, reg_hi, 0x00])
    cs      = checksum(payload)
    return f":{payload[0]:02X}{payload[1]:02X}{payload[2]:02X}{payload[3]:02X}{cs:02X}\n".encode()

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
    if not line.startswith(':'):
        return None
    try:
        raw = bytes.fromhex(line[1:])
    except ValueError:
        return None
    if len(raw) < 5:
        return None
    if sum(raw) & 0xFF != 0x55:
        return None
    cmd       = raw[0]
    reg       = raw[1] | (raw[2] << 8)
    flags     = raw[3]
    val_bytes = raw[4:-1]
    return cmd, reg, flags, val_bytes

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
        timeout=0.1   # short timeout so we loop ourselves
    )

def read_hex_response(ser: serial.Serial, expected_cmd: int,
                      expected_reg: int, timeout: float = 2.0) -> Optional[tuple]:
    """
    Read lines until we see a HEX response matching cmd+reg, or timeout.
    TEXT protocol lines (not starting with ':') are discarded silently.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode('ascii', errors='ignore').strip()
        if not line.startswith(':'):
            continue                     # discard TEXT frame, keep waiting
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
                return None             # unknown register on this firmware
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
        raw = int.from_bytes(val_bytes, 'little', signed=True)
    except Exception:
        return val_bytes.hex()
    if reg.fmt == "h":
        return f"0x{raw & 0xFFFFFFFF:04X}"
    if reg.choices:
        label = reg.choices.get(raw, f"unknown ({raw})")
        return f"{raw} — {label}"
    scaled = raw * reg.scale
    unit   = f" {reg.unit}" if reg.unit else ""
    return f"{scaled:.2f}{unit}" if reg.scale < 1 else f"{scaled:.0f}{unit}"

def scaled_to_raw(reg: Register, value: float) -> int:
    return round(value / reg.scale)

# ─────────────────────────────────────────────
#  Read all
# ─────────────────────────────────────────────

def print_header(port: str):
    print(f"\n{B}{CYN}{'─'*64}{R}")
    print(f"{B}{CYN}  Victron MPPT  ·  VE.Direct HEX Tool  ·  {port}{R}")
    print(f"{B}{CYN}{'─'*64}{R}\n")

def read_all(ser: serial.Serial, port: str):
    print_header(port)
    for section_name, addresses in SECTIONS:
        print(f"  {B}{YLW}▸ {section_name}{R}")
        print(f"  {DIM}{'─'*60}{R}")
        any_printed = False
        for addr in addresses:
            reg = REG_BY_ADDR.get(addr)
            if not reg:
                continue
            val_bytes = send_get(ser, addr)
            if val_bytes is None:
                continue                # not supported on this firmware, skip
            value_str = decode_value(reg, val_bytes)
            rw_badge  = f"{GRN}R/W{R}" if "W" in reg.rw else f"{DIM} R {R}"
            addr_str  = f"{DIM}0x{addr:04X}{R}"
            print(f"  {rw_badge}  {addr_str}  {reg.name:<28} {B}{value_str}{R}")
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
        print(f"  Writing {new_value} {reg.unit}  →  raw 0x{raw_value & 0xFFFF:04X} …",
              end=" ", flush=True)
        ok = send_set(ser, reg.address, raw_value)
        if ok:
            print(f"{GRN}OK{R}")
            try:
                if input(f"  Save to NVM? (y/n): ").strip().lower() == 'y':
                    print(f"  Saving …", end=" ", flush=True)
                    print(f"{GRN}OK{R}" if nvm_save(ser) else f"{RED}Failed{R}")
            except (KeyboardInterrupt, EOFError):
                print(); break
        else:
            print(f"{RED}Failed{R}  (register may reject this value)")
        print()

# ─────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="VE.Direct HEX Tool — read/write Victron MPPT settings")
    parser.add_argument("port",    nargs="?",
                        help="Serial port (auto-detected if omitted)")
    parser.add_argument("--write", "-w", action="store_true",
                        help="Interactive write mode")
    args = parser.parse_args()

    port = args.port or find_port()
    if not port:
        print(f"{RED}No VE.Direct device found. Specify port manually:{R}")
        print(f"  python3 vedirect_hex.py /dev/tty.usbserial-XXXXX")
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
