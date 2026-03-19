#!/usr/bin/env python3
"""
VE.Direct TUI — Victron MPPT terminal interface
v6 — Algorithm selector sets battery type to match preset (not user defined)

Tabs:
  (L)ive     — refreshes from TEXT protocol stream
  (C)harger  — charger data via HEX
  (H)istory  — history registers via HEX
  (S)ettings — battery settings, type a number + Enter to edit

Navigation:
  Tab / Left / Right arrow  — switch tabs
  L C H S                   — jump to tab by letter
  Up / Down arrow           — scroll within tab
  R                         — refresh current tab (HEX tabs only)
  Q / Ctrl-C                — quit

On Settings tab:
  Type a register number + Enter to edit it
  ESC clears the number buffer

Usage:
    python3 vedirect_tui.py                        # auto-detect port
    python3 vedirect_tui.py /dev/tty.usbserial-XXX # specify port
"""

import curses
import serial
import serial.tools.list_ports
import sys
import time
import threading
import argparse
from dataclasses import dataclass, field
from typing import Optional

# ─────────────────────────────────────────────
#  TEXT protocol maps
# ─────────────────────────────────────────────
CS_STATES = {
    "0":"Off", "2":"Fault", "3":"Bulk", "4":"Absorption",
    "5":"Float", "6":"Storage", "7":"Equalize (manual)",
    "245":"Wake-up", "247":"Auto equalize", "250":"Blocked",
    "252":"External control", "255":"Unavailable",
}
MPPT_STATES = {
    "0":"Off", "1":"Voltage/current limited", "2":"MPP tracker active",
}
ERR_CODES = {
    "0":"No error", "2":"Battery voltage too high",
    "17":"Charger temp too high", "18":"Charger over-current",
    "19":"Charger current reversed", "20":"Bulk time limit exceeded",
    "21":"Current sensor issue", "26":"Terminals overheated",
    "28":"Power stage issue", "33":"Input voltage too high",
    "34":"Input current too high", "38":"Input shutdown (excess bat V)",
    "39":"Input shutdown (bat I during off)",
}

# ─────────────────────────────────────────────
#  HEX Register definitions
# ─────────────────────────────────────────────
@dataclass
class Register:
    address: int
    name:    str
    desc:    str
    unit:    str
    scale:   float
    rw:      str
    fmt:     str  = "d"
    size:    int  = 2
    choices: dict = field(default_factory=dict)

REGISTERS = [
    # Charger data
    Register(0xEDD5, "Charger Voltage",    "Voltage at battery terminals",         "V",    0.01,  "R",  "d", 2),
    Register(0xEDD7, "Charger Current",    "Battery + load current combined",      "A",    0.1,   "R",  "d", 2),
    Register(0xEDAD, "Load Current",       "Load output current",                  "A",    0.1,   "R",  "d", 2),
    Register(0xEDDB, "Internal Temp",      "Charger internal temperature",         "°C",   0.01,  "R",  "d", 2),
    Register(0xEDDA, "Charger Error",      "Charger error code",                   "",     1,     "R",  "d", 1,
             {0:"No error", 2:"Battery voltage too high",
              17:"Charger temp too high", 18:"Charger over-current",
              19:"Charger current reversed", 20:"Bulk time limit exceeded",
              21:"Current sensor issue", 26:"Terminals overheated",
              28:"Power stage issue", 33:"Input voltage too high",
              34:"Input current too high", 38:"Input shutdown (excess bat V)",
              39:"Input shutdown (bat I during off)", 65:"Lost comms with temp sensor",
              67:"Charger not calibrated", 68:"BMS lockout",
              116:"Calibration data lost", 119:"Settings invalid"}),
    Register(0xEDD4, "Charger State Info", "Additional charger state info",        "",     1,     "R",  "b", 1),
    Register(0x010A, "Serial Number",      "Device serial number",                 "",     1,     "R",  "s", 0),
    Register(0x010B, "Model Name",         "Model name string",                    "",     1,     "R",  "s", 0),
    Register(0x0140, "Capabilities",       "Device capabilities bitmask",          "",     1,     "R",  "cap", 4),
    # History
    Register(0xEDDD, "System Yield",       "Total system yield (non-resettable)",  "kWh",  0.01,  "R",  "d", 4),
    Register(0xEDDC, "User Yield",         "User yield (resettable)",              "kWh",  0.01,  "R",  "d", 4),
    Register(0xEDD3, "Yield Today",        "Yield today",                          "Wh",   10,    "R",  "d", 2),
    Register(0xEDD2, "Max Power Today",    "Maximum power today",                  "W",    1,     "R",  "d", 2),
    Register(0xEDD1, "Yield Yesterday",    "Yield yesterday",                      "Wh",   10,    "R",  "d", 2),
    Register(0xEDD0, "Max Pwr Yesterday",  "Maximum power yesterday",              "W",    1,     "R",  "d", 2),
    # Battery settings
    Register(0xEDF0, "Max Charge Current", "Maximum battery charge current",       "A",    0.1,   "RW", "d", 2),
    Register(0xEDF1, "Battery Type",       "Battery type (0xFF=user defined)",     "",     1,     "RW", "d", 1,
             {0:"User defined", 1:"Gel long life / OPzV",
              2:"Gel / AGM deep discharge", 3:"Default / Gel / AGM DD",
              4:"AGM spiral / OPzS / Rolls AGM", 5:"PzS traction / OPzS (low)",
              6:"PzS traction / OPzS (mid)", 7:"PzS traction / OPzS (high)",
              8:"LiFePO4", 255:"User defined (custom)"}),
    Register(0xEDF2, "Temp Compensation",  "Battery temp compensation",            "mV/K", 0.01,  "RW", "d", 2),
    Register(0xEDF4, "Equalise Voltage",   "Equalisation voltage",                 "V",    0.01,  "RW", "d", 2),
    Register(0xEDF6, "Float Voltage",      "Float phase target voltage",           "V",    0.01,  "RW", "d", 2),
    Register(0xEDF7, "Absorption Voltage", "Absorption phase target voltage",      "V",    0.01,  "RW", "d", 2),
    Register(0xEDEF, "Battery Voltage",    "System battery voltage (operational)", "V",    1,     "R",  "d", 1),
    Register(0xEDEA, "Battery V Setting",  "Battery voltage setting (0=AUTO, 12=12V, 24=24V)", "V", 1, "RW", "d", 1,
             {0:"Auto", 12:"12V", 24:"24V"}),
    Register(0xEDFB, "Absorption Time",    "Max absorption time",                  "h",    0.01,  "RW", "d", 2),
    Register(0xEDFC, "Bulk Time Limit",    "Maximum bulk charge time",             "h",    0.01,  "RW", "d", 2),
    Register(0xEDFD, "Auto Equalise",      "Auto equalisation interval (0=off)",   "days", 1,     "RW", "d", 1),
    Register(0xEDFE, "Adaptive Mode",      "Adaptive absorption mode",             "",     1,     "RW", "d", 1,
             {0:"Off", 1:"On"}),
    Register(0xED2E, "Re-bulk V Offset",   "Re-bulk voltage offset",               "V",    0.01,  "RW", "d", 2),
    # Load output
    Register(0xEDA8, "Load Mode",          "Load output control mode",             "",     1,     "RW", "d", 1,
             {0:"Always off", 1:"BattLife algorithm", 2:"Conventional alg 1",
              3:"Conventional alg 2", 4:"Always on",
              5:"User defined alg 1", 6:"User defined alg 2"}),
    Register(0xEDA9, "Load Low V Off",     "Load switch-off voltage",              "V",    0.01,  "RW", "d", 2),
    Register(0xEDAA, "Load Low V On",      "Load switch-on voltage",               "V",    0.01,  "RW", "d", 2),
]

REG_BY_ADDR = {r.address: r for r in REGISTERS}

TABS = [
    ("ive",     "L", []),
    ("harger",  "C", [0xEDD5, 0xEDD7, 0xEDAD, 0xEDDB, 0xEDDA, 0xEDD4,
                       0x010A, 0x010B, 0x0140]),
    ("istory",  "H", [0xEDDD, 0xEDDC, 0xEDD3, 0xEDD2, 0xEDD1, 0xEDD0]),
    ("ettings", "S", [0xEDF0, 0xEDF1, 0xEDF7, 0xEDF6, 0xEDF4, 0xEDF2,
                       0xEDEA, 0xEDFB, 0xEDFC, 0xEDFD, 0xEDFE,
                       0xED2E, 0xEDA8, 0xEDA9, 0xEDAA,
                       0xEDEF]),
]
TAB_KEYS = {t[1].lower(): i for i, t in enumerate(TABS)}

# ─────────────────────────────────────────────
#  Charging algorithm definitions
#  Based on BlueSolar MPPT 100/30 & 100/50 manual Rev 07
#  All voltages are 12V values; 24V = × 2
# ─────────────────────────────────────────────

@dataclass
class Algorithm:
    position:    int
    name:        str
    desc:        str
    absorption:  float        # V (12V)
    float_v:     float        # V (12V)
    equalize:    float        # V (12V), 0 = n/a
    eq_current:  int          # % of max current, 0 = n/a
    temp_comp:   float        # mV/°C, 0 = disabled
    is_lithium:  bool = False
    is_custom:   bool = False

ALGORITHMS = [
    Algorithm(0, "Gel long life / OPzV",
              "Gel Exide A600, Gel MK, Gel long life OPzV",
              14.1, 13.8, 15.9, 8,  -16),
    Algorithm(1, "Gel / AGM deep discharge",
              "Gel Victron DD, Gel Exide A200, AGM Victron DD, OPzS",
              14.3, 13.8, 16.1, 8,  -16),
    Algorithm(2, "Default / Gel / AGM DD",
              "DEFAULT. Gel Victron DD, Gel Exide A200, AGM Victron DD, OPzS",
              14.4, 13.8, 16.2, 8,  -16),
    Algorithm(3, "AGM spiral / OPzS / Rolls AGM",
              "AGM spiral cell, Stationary tubular plate OPzS, Rolls AGM",
              14.7, 13.8, 16.5, 8,  -16),
    Algorithm(4, "PzS traction / OPzS (low)",
              "PzS tubular plate traction batteries, OPzS batteries",
              14.9, 13.8, 16.7, 25, -16),
    Algorithm(5, "PzS traction / OPzS (mid)",
              "PzS tubular plate traction batteries, OPzS batteries",
              15.1, 13.8, 16.9, 25, -16),
    Algorithm(6, "PzS traction / OPzS (high)",
              "PzS tubular plate traction batteries, OPzS batteries",
              15.3, 13.8, 17.1, 25, -16),
    Algorithm(7, "LiFePO4",
              "Lithium Iron Phosphate batteries — equalization N/A",
              14.2, 13.5, 0,    0,  0, is_lithium=True),
    Algorithm(8, "Custom / User Defined",
              "All parameters fully user-configurable",
              0,    0,    0,    0,  0, is_custom=True),
]

ALG_BY_POS = {a.position: a for a in ALGORITHMS}

# Registers locked (n/a) when LiFePO4 is selected
LITHIUM_LOCKED = {
    0xEDF4,   # Equalise Voltage
    0xEDFD,   # Auto Equalise
    0xEDF2,   # Temp Compensation
    0xEDFE,   # Adaptive Mode  (should be fixed time for LiFePO4)
}

# Battery V Setting - locked to 12 or 24 only
BATTERY_V_SETTINGS = {12, 24}

# Safe voltage ranges per register for LiFePO4 (min, max) in display units
LITHIUM_RANGES = {
    0xEDF7: (14.0, 14.6),   # Absorption Voltage
    0xEDF6: (13.2, 13.8),   # Float Voltage
}

# Safe voltage ranges for lead-acid (all algorithms 0-6) (min, max)
LEADACID_RANGES = {
    0xEDF7: (13.0, 16.0),   # Absorption Voltage
    0xEDF6: (12.5, 14.5),   # Float Voltage
    0xEDF4: (14.0, 17.5),   # Equalise Voltage
}

def get_system_voltage(hex_cache: dict) -> int:
    """Return system voltage (12 or 24) based on Battery V Setting register."""
    vset_str = hex_cache.get(0xEDEA, "")
    try:
        # Handle values like "12V", "24 V", "Auto", "12", "24", etc.
        vset_str = vset_str.strip()
        # Remove 'V' suffix if present
        vset_str = vset_str.rstrip('V').strip()
        # Check for Auto first
        if vset_str.lower() == "auto" or vset_str == "0":
            # AUTO - try to detect from battery voltage
            vbat_str = hex_cache.get(0xEDEF, "")
            if vbat_str:
                try:
                    vbat = float(vbat_str.split()[0])
                    if vbat >= 20:
                        return 24
                except (ValueError, IndexError):
                    pass
            return 12
        # Try to extract numeric value
        vset = int(float(vset_str))
        if vset in (12, 24):
            return vset
        # If some other value, try to infer from it
        if vset >= 20:
            return 24
        return 12
    except (ValueError, IndexError, AttributeError):
        # Default to 12V if we can't determine
        return 12


def get_current_algorithm(hex_cache: dict) -> Optional[Algorithm]:
    """
    Infer current algorithm from cached register values.
    First checks battery type register - if set to a preset (0-8), use that.
    Otherwise matches absorption and float voltages against known presets.
    Returns Custom only if no match found.
    """
    # First check battery type register
    btype_str = hex_cache.get(0xEDF1, "")
    try:
        btype = int(btype_str.split()[0])
        # Register values 1-8 map to algorithm positions 0-7
        # Register value 0 = User defined (not an algorithm)
        # Register value 255 = User defined (custom)
        if 1 <= btype <= 8:
            alg_pos = btype - 1  # Convert register value to algorithm position
            return ALG_BY_POS.get(alg_pos)
    except (ValueError, IndexError):
        pass

    # If battery type is 255 (user defined), infer from voltages
    abs_str   = hex_cache.get(0xEDF7, "")
    float_str = hex_cache.get(0xEDF6, "")

    def parse_v(s):
        try:
            return float(s.split()[0])
        except Exception:
            return None

    abs_v   = parse_v(abs_str)
    float_v = parse_v(float_str)

    if abs_v is None or float_v is None:
        return None

    for alg in ALGORITHMS:
        if alg.is_custom:
            continue
        if abs(abs_v - alg.absorption) < 0.05 and abs(float_v - alg.float_v) < 0.05:
            return alg

    return ALGORITHMS[8]  # custom / user defined


def get_battery_type_display(hex_cache: dict) -> str:
    """
    Returns a human-readable battery type string.
    If type is 255 (user defined), infer algorithm from voltages.
    """
    btype_str = hex_cache.get(0xEDF1, "")
    try:
        btype = int(btype_str.split()[0])
    except Exception:
        return btype_str

    if btype != 255:
        # Battery type register - map to algorithm name
        # Register 0xEDF1 choices: 0=User defined, 1-8 are presets
        ALG_NAMES = {
            0: "User defined",
            1: "Gel long life / OPzV",
            2: "Gel / AGM deep discharge",
            3: "Default / Gel / AGM DD",
            4: "AGM spiral / OPzS",
            5: "PzS traction / OPzS (low)",
            6: "PzS traction / OPzS (mid)",
            7: "PzS traction / OPzS (high)",
            8: "LiFePO4",
        }
        name = ALG_NAMES.get(btype, f"Preset {btype}")
        # If it's "User defined" (btype=0), we need to show it properly
        # Don't fall through to voltage detection for btype=0
        if btype == 0:
            return name
        return name

    # Type 255 = user defined — infer from voltages
    alg = get_current_algorithm(hex_cache)
    if alg is None:
        return "User defined (detecting...)"
    if alg.is_custom:
        return "User defined (custom)"
    return f"User defined — matches pos {alg.position}: {alg.name}"

def is_register_locked(addr: int, alg: Optional[Algorithm]) -> bool:
    """Return True if this register should be blocked for the current algorithm."""
    # Battery V Setting is locked (must use algorithm selector to change)
    # Users can only change via algorithm presets
    if addr == 0xEDEA:
        return True
    if alg is None or alg.is_custom:
        return False
    if alg.is_lithium and addr in LITHIUM_LOCKED:
        return True
    return False

def get_range_for_register(addr: int, alg: Optional[Algorithm], system_voltage: int = 12) -> Optional[tuple]:
    """Return (min, max) safety range for a register given current algorithm and system voltage."""
    # Battery V Setting is locked to 12 or 24 only
    if addr == 0xEDEA:
        return (12, 24)
    
    # Get base ranges
    if alg is None or alg.is_custom:
        return None
    if alg.is_lithium:
        ranges = LITHIUM_RANGES.get(addr)
    else:
        ranges = LEADACID_RANGES.get(addr)
    
    if ranges is None:
        return None
    
    # Scale ranges for 24V systems (ranges are defined for 12V)
    if system_voltage == 24:
        return (ranges[0] * 2, ranges[1] * 2)
    return ranges

# ─────────────────────────────────────────────
#  Serial access — single worker thread
# ─────────────────────────────────────────────
# All serial I/O goes through a single worker thread via a job queue.
# The TEXT reader and HEX fetcher never touch the port simultaneously.

import queue

class SerialWorker:
    """
    Single thread that owns the serial port.
    Clients submit jobs; results come back via per-job Events.
    """
    def __init__(self, ser: serial.Serial):
        self.ser        = ser
        self._q         = queue.Queue()
        self._stop      = threading.Event()
        self._text_buf  = {}          # partial TEXT frame being assembled
        self._text_ready = queue.Queue()  # completed TEXT frames
        self._t         = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def stop(self):
        self._stop.set()

    # ── public API ──────────────────────────

    def read_text_frame(self, timeout=15.0) -> Optional[dict]:
        """Block until one complete TEXT frame is available."""
        try:
            return self._text_ready.get(timeout=timeout)
        except queue.Empty:
            return None

    def ping(self):
        ev = threading.Event()
        self._q.put(('ping', ev))
        ev.wait(1.0)

    def get_register(self, address: int, retries=3) -> Optional[bytes]:
        result = {}
        ev     = threading.Event()
        self._q.put(('get', address, retries, result, ev))
        ev.wait(15.0)  # Increased timeout for slower devices
        return result.get('val')

    def set_register(self, address: int, raw_value: int,
                     size: int = 2) -> tuple:
        result = {}
        ev     = threading.Event()
        self._q.put(('set', address, raw_value, size, result, ev))
        ev.wait(10.0)
        return result.get('ok', False), result.get('flags')

    def nvm_save(self) -> bool:
        # NVM save needs extra care - ping first, wait, then save
        for attempt in range(5):
            # Ping to keep HEX session alive
            self.ping()
            time.sleep(0.5)
            
            # Try to save
            ok, flags = self.set_register(0xEB99, 1, size=1)
            if ok:
                return True
            
            # Wait before retry
            time.sleep(1.0)
        return False

    # ── internal ────────────────────────────

    def _run(self):
        while not self._stop.is_set():
            # Always drain any waiting serial bytes first, feeding TEXT parser
            self._drain_serial()
            # Then process one queued HEX job if available
            try:
                job = self._q.get(timeout=0.02)
            except queue.Empty:
                continue
            try:
                self._dispatch(job)
            except Exception:
                pass

    def _drain_serial(self):
        """Read all immediately available lines, parse TEXT frames on the fly."""
        while True:
            raw = self.ser.readline()
            if not raw:
                break
            line = raw.decode('ascii', errors='ignore').strip()
            if not line:
                continue
            # HEX lines are not TEXT — skip them here (HEX responses are
            # read synchronously inside _send_get/_send_set)
            if line.startswith(':'):
                continue
            if '\t' in line:
                key, _, value = line.partition('\t')
                key = key.strip(); value = value.strip()
                if key == 'Checksum':
                    if self._text_buf:
                        self._text_ready.put(dict(self._text_buf))
                        self._text_buf = {}
                else:
                    self._text_buf[key] = value
            elif line == 'Checksum':
                if self._text_buf:
                    self._text_ready.put(dict(self._text_buf))
                    self._text_buf = {}

    def _dispatch(self, job):
        kind = job[0]
        if kind == 'ping':
            _, ev = job
            self._ping()
            ev.set()
        elif kind == 'get':
            _, address, retries, result, ev = job
            val = self._send_get(address, retries)
            result['val'] = val
            ev.set()
        elif kind == 'set':
            _, address, raw_value, size, result, ev = job
            ok, flags = self._send_set(address, raw_value, size)
            result['ok']    = ok
            result['flags'] = flags
            ev.set()

    # ── low-level serial ops ─────────────────

    CHAR_DELAY = 0.001  # 1ms inter-character delay for Bus Pirate FIFO

    def _write_slow(self, data: bytes):
        """Write byte-by-byte with inter-character delay."""
        for byte in data:
            self.ser.write(bytes([byte]))
            time.sleep(self.CHAR_DELAY)

    def _ping(self):
        self._write_slow(b':154\n')
        time.sleep(0.1)

    def _read_hex_response(self, expected_cmd, expected_reg,
                           timeout=2.0) -> Optional[tuple]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode('ascii', errors='ignore').strip()
            if not line.startswith(':'):
                continue
            result = self._parse_hex(line)
            if result is None:
                continue
            cmd, reg, flags, val_bytes = result
            if cmd == expected_cmd and reg == expected_reg:
                return result
        return None

    @staticmethod
    def _parse_hex(line: str) -> Optional[tuple]:
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

    @staticmethod
    def _build_get(register: int) -> bytes:
        reg_lo = register & 0xFF
        reg_hi = (register >> 8) & 0xFF
        cs = (0x55 - (0x07 + reg_lo + reg_hi)) & 0xFF
        return f":7{reg_lo:02X}{reg_hi:02X}{cs:02X}\n".encode()

    @staticmethod
    def _build_set(register: int, value: int, size: int) -> bytes:
        reg_lo    = register & 0xFF
        reg_hi    = (register >> 8) & 0xFF
        mask      = (1 << (size * 8)) - 1
        val_bytes = (value & mask).to_bytes(size, 'little')
        data      = bytes([reg_lo, reg_hi, 0x00]) + val_bytes
        cs        = (0x55 - (0x08 + sum(data))) & 0xFF
        hex_str   = "".join(f"{b:02X}" for b in data)
        return f":8{hex_str}{cs:02X}\n".encode()

    def _send_get(self, register: int, retries=3) -> Optional[bytes]:
        for _ in range(retries):
            self.ser.reset_input_buffer()
            self._write_slow(self._build_get(register))
            result = self._read_hex_response(0x07, register)
            if result is not None:
                cmd, reg, flags, val_bytes = result
                if flags & 0x01:
                    return None
                return val_bytes
        return None

    def _send_set(self, register: int, raw_value: int,
                  size: int) -> tuple:
        self.ser.reset_input_buffer()
        self._write_slow(self._build_set(register, raw_value, size))
        # wait specifically for SET response, ignore async frames
        deadline = time.time() + 4.0
        while time.time() < deadline:
            raw = self.ser.readline()
            if not raw: continue
            line = raw.decode('ascii', errors='ignore').strip()
            if not line.startswith(':'): continue
            result = self._parse_hex(line)
            if result is None: continue
            cmd, reg, flags, val_bytes = result
            if cmd in (0x08, 0x04) and reg == register:
                if cmd == 0x04:
                    return False, 0x04
                return flags == 0x00, flags
        return False, None


# ─────────────────────────────────────────────
#  Value decode
# ─────────────────────────────────────────────

def decode_value(reg: Register, val_bytes: bytes) -> str:
    if not val_bytes:
        return "no data"
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
    if reg.fmt == "cap":
        # Capabilities bitmask — spec Rev 18 register 0x0140
        CAP_BITS = {
            0:  "Load output",
            1:  "Rotary encoder",
            2:  "History",
            3:  "Batterysafe mode",
            4:  "Adaptive mode",
            5:  "Manual equalise",
            6:  "Auto equalise",
            7:  "Storage mode",
            8:  "Remote on/off",
            9:  "Solar timer/streetlight",
            10: "Alt TX pin function",
            11: "User defined load switch",
            12: "Load current in TEXT",
            13: "Panel current",
            14: "BMS support",
            15: "External control",
            16: "Sync charging",
            17: "Alarm relay",
            18: "Alt RX pin function",
            19: "Virtual load output",
            20: "Virtual relay",
            21: "Plugin display",
            25: "Load AES",
            26: "Battery test",
            27: "PAYGO",
        }
        if raw == 0:
            return "none"
        active = [CAP_BITS.get(i, f"bit{i}") for i in range(28) if raw & (1 << i)]
        return ", ".join(active)
        # 0xEDD4 charger state info bits per spec Rev 18
        BITS = {
            0: "Batterysafe",
            1: "Auto equalise",
            2: "Manual equalise",
            3: "Low battery",
        }
        if raw == 0:
            return "Normal (no flags)"
        active = [BITS.get(i, f"bit{i}") for i in range(8) if raw & (1 << i)]
        return ", ".join(active)
    if reg.choices:
        label = reg.choices.get(raw, f"unknown ({raw})")
        return f"{label}"
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
#  TEXT frame parser
# ─────────────────────────────────────────────

def parse_text_frame(frame: dict) -> list:
    rows = []
    if 'VPV' in frame:
        rows.append(("Solar Voltage",    f"{int(frame['VPV'])/1000:.3f} V"))
    if 'PPV' in frame:
        rows.append(("Solar Power",      f"{frame['PPV']} W"))
    if 'IL' in frame:
        rows.append(("Solar Current",    f"{int(frame['IL'])/1000:.3f} A"))
    if 'V' in frame:
        rows.append(("Battery Voltage",  f"{int(frame['V'])/1000:.3f} V"))
    if 'I' in frame:
        rows.append(("Battery Current",  f"{int(frame['I'])/1000:.3f} A"))
    if 'VS' in frame:
        rows.append(("Starter Voltage",  f"{int(frame['VS'])/1000:.3f} V"))
    if 'CS' in frame:
        rows.append(("Charge State",     CS_STATES.get(frame['CS'], f"Unknown ({frame['CS']})")))
    if 'MPPT' in frame:
        rows.append(("MPPT State",       MPPT_STATES.get(frame['MPPT'], f"Unknown ({frame['MPPT']})")))
    if 'ERR' in frame:
        rows.append(("Error",            ERR_CODES.get(frame['ERR'], f"Error {frame['ERR']}")))
    if 'LOAD' in frame:
        rows.append(("Load Output",      frame['LOAD']))
    if 'H19' in frame:
        rows.append(("Yield Total",      f"{int(frame['H19'])/100:.2f} kWh"))
    if 'H20' in frame:
        rows.append(("Yield Today",      f"{int(frame['H20'])*10} Wh"))
    if 'H21' in frame:
        rows.append(("Max Power Today",  f"{frame['H21']} W"))
    if 'H22' in frame:
        rows.append(("Yield Yesterday",  f"{int(frame['H22'])*10} Wh"))
    if 'H23' in frame:
        rows.append(("Max Pwr Yest.",    f"{frame['H23']} W"))
    if 'HSDS' in frame:
        rows.append(("Day Sequence",     frame['HSDS']))
    if 'FW' in frame:
        rows.append(("Firmware",         f"{int(frame['FW'])/100:.2f}"))
    if 'PID' in frame:
        rows.append(("Product ID",       frame['PID']))
    if 'SER#' in frame:
        rows.append(("Serial Number",    frame['SER#']))
    return rows

# ─────────────────────────────────────────────
#  Shared state
# ─────────────────────────────────────────────

class State:
    def __init__(self):
        self.lock         = threading.Lock()
        self.live_rows    = []
        self.live_updated = 0.0
        self.hex_cache    = {}
        self.hex_loading  = set()
        self.hex_retries  = {}
        self.write_log    = []
        self.graph_data   = []     # list of (day_offset, yield_wh, max_power_w)
        self.graph_loading = False
        self.graph_updated = 0.0

# ─────────────────────────────────────────────
#  Background threads
# ─────────────────────────────────────────────

def live_reader_thread(worker: SerialWorker, state: State,
                       stop_event: threading.Event):
    """Continuously reads TEXT frames via the serial worker."""
    while not stop_event.is_set():
        frame = worker.read_text_frame(timeout=15.0)
        if frame:
            rows = parse_text_frame(frame)
            with state.lock:
                state.live_rows    = rows
                state.live_updated = time.time()

def hex_fetch_thread(worker: SerialWorker, state: State,
                     addresses: list, stop_event: threading.Event):
    """Fetches HEX registers sequentially via the serial worker."""
    worker.ping()
    for addr in addresses:
        if stop_event.is_set():
            break
        reg = REG_BY_ADDR.get(addr)
        if not reg:
            continue
        with state.lock:
            state.hex_loading.add(addr)
        worker.ping()
        val_bytes = worker.get_register(addr)
        value_str = decode_value(reg, val_bytes) if val_bytes is not None else "—"
        with state.lock:
            state.hex_cache[addr] = value_str
            state.hex_loading.discard(addr)

def auto_refetch_thread(worker: SerialWorker, state: State,
                        all_addresses: list, stop_event: threading.Event):
    """
    Periodically retries addresses that returned '—'.
    After 3 failed attempts marks them 'unavailable' and stops retrying
    until a manual refresh resets the retry count.
    """
    MAX_RETRIES = 3
    while not stop_event.is_set():
        for _ in range(50):
            if stop_event.is_set():
                return
            time.sleep(0.1)

        with state.lock:
            missing = [a for a in all_addresses
                       if state.hex_cache.get(a) == "—"
                       and a not in state.hex_loading
                       and state.hex_retries.get(a, 0) < MAX_RETRIES]

        if not missing:
            continue

        worker.ping()
        for addr in missing:
            if stop_event.is_set():
                return
            reg = REG_BY_ADDR.get(addr)
            if not reg:
                continue
            with state.lock:
                state.hex_loading.add(addr)
            worker.ping()
            val_bytes = worker.get_register(addr)
            with state.lock:
                state.hex_loading.discard(addr)
                if val_bytes is not None:
                    state.hex_cache[addr]   = decode_value(reg, val_bytes)
                    state.hex_retries[addr] = 0
                else:
                    state.hex_retries[addr] = state.hex_retries.get(addr, 0) + 1
                    if state.hex_retries[addr] >= MAX_RETRIES:
                        state.hex_cache[addr] = "unavailable"

def graph_fetch_thread(worker: SerialWorker, state: State,
                       stop_event: threading.Event):
    """
    Fetches daily history from registers 0x1050 onwards.
    Confirmed layout: byte[1]=yield×10Wh, byte[30]=max power W.
    Re-pings every 3 registers to keep HEX mode alive.
    0x1050 = today (may be incomplete), 0x1051 = yesterday, etc.
    Uses longer timeouts and more retries to ensure we get all 30 days.
    """
    with state.lock:
        state.graph_loading = True

    worker.ping()
    time.sleep(1.0)  # Longer initial delay for device to respond
    days = []
    consecutive_failures = 0
    MAX_CONSECUTIVE_FAILURES = 8  # Allow up to 8 empty registers before stopping (covers 30 days)

    for i in range(31):
        if stop_event.is_set():
            break

        # Re-ping every 3 to keep HEX mode alive (more frequent)
        if i > 0 and i % 3 == 0:
            worker.ping()
            time.sleep(0.3)

        addr = 0x1050 + i
        # Use more retries and longer timeout for history registers
        val_bytes = worker.get_register(addr, retries=5)

        if val_bytes is None or len(val_bytes) < 31:
            consecutive_failures += 1
            if consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                # Stop after multiple consecutive failures
                break
            continue  # Try next day
        else:
            consecutive_failures = 0

        yield_wh  = val_bytes[1] * 10   # byte[1] × 10Wh
        max_power = val_bytes[30]        # byte[30] in W
        days.append((i, yield_wh, max_power))

    with state.lock:
        state.graph_data    = days
        state.graph_loading = False
        state.graph_updated = time.time()


def draw_graph_tab(win, state: State, rows: int, cols: int, start_row: int = 2):
    """Draw a bar chart of daily yield (Wh). start_row allows embedding in History tab."""
    with state.lock:
        data    = list(state.graph_data)
        loading = state.graph_loading
        updated = state.graph_updated

    if loading:
        try:
            win.addstr(start_row + 1, 2, "Loading history...",
                       curses.color_pair(C_DIM))
        except curses.error:
            pass
        return

    if not data:
        try:
            win.addstr(start_row + 1, 2, "No history data — press R to fetch",
                       curses.color_pair(C_DIM))
        except curses.error:
            pass
        return

    # Title
    age_str = "  Daily yield history  |  " + \
              (f"updated {time.time()-updated:.0f}s ago" if updated else "press R to load")
    try:
        win.addstr(start_row, 0, age_str[:cols],
                   curses.color_pair(C_HEADER) | curses.A_BOLD)
    except curses.error:
        pass

    # Chart dimensions
    chart_top    = start_row + 1
    chart_bottom = rows - 5
    chart_height = chart_bottom - chart_top
    chart_left   = 8
    chart_right  = cols - 2
    chart_width  = chart_right - chart_left

    if chart_height < 4 or chart_width < 10:
        return

    # Skip today (index 0) if yield is 0 — it's incomplete
    plot_data = data[1:] if data and data[0][1] == 0 else data
    if not plot_data:
        return

    max_wh = max(d[1] for d in plot_data) or 1

    bar_count = min(len(plot_data), chart_width // 3)
    bar_width = max(1, chart_width // bar_count - 1)
    visible   = plot_data[:bar_count]

    # Y axis labels
    for pct in [0, 25, 50, 75, 100]:
        y_val = int(max_wh * pct / 100)
        y_row = chart_bottom - int(chart_height * pct / 100)
        if chart_top <= y_row <= chart_bottom:
            try:
                win.addstr(y_row, 0, f"{y_val:>5}Wh", curses.color_pair(C_DIM))
                win.addstr(y_row, chart_left - 1, "┤", curses.color_pair(C_DIM))
            except curses.error:
                pass

    # Axes
    for r in range(chart_top, chart_bottom + 1):
        try:
            win.addstr(r, chart_left - 1, "│", curses.color_pair(C_DIM))
        except curses.error:
            pass
    try:
        win.addstr(chart_bottom + 1, chart_left - 1,
                   "└" + "─" * (chart_right - chart_left + 1),
                   curses.color_pair(C_DIM))
    except curses.error:
        pass

    # Bars
    for i, (day_offset, yield_wh, max_power) in enumerate(visible):
        bar_h    = max(1, int(chart_height * yield_wh / max_wh)) if yield_wh > 0 else 0
        bar_x    = chart_left + i * (bar_width + 1)
        # Cycle through 4 shades of green for subtle gradient effect
        green_attrs = [
            curses.color_pair(C_GREEN_1) | curses.A_BOLD,
            curses.color_pair(C_GREEN_2) | curses.A_BOLD,
            curses.color_pair(C_GREEN_3) | curses.A_BOLD,
            curses.color_pair(C_GREEN_4) | curses.A_BOLD,
        ]
        bar_attr = green_attrs[i % 4]

        for r in range(bar_h):
            bar_row = chart_bottom - r
            if chart_top <= bar_row < chart_bottom + 1:
                try:
                    win.addstr(bar_row, bar_x, "█" * bar_width, bar_attr)
                except curses.error:
                    pass

        # Day label
        label = f"-{day_offset}d"
        try:
            win.addstr(chart_bottom + 2, bar_x, label[:bar_width + 1],
                       curses.color_pair(C_DIM))
        except curses.error:
            pass

        # Value on top if room
        val_str = f"{yield_wh}Wh"
        if bar_h >= 2 and bar_width >= len(val_str):
            try:
                win.addstr(chart_bottom - bar_h, bar_x, val_str,
                           curses.color_pair(C_UPDATED))
            except curses.error:
                pass

    # B to go back hint
    try:
        win.addstr(rows - 2, 2, "B:Back to history list  R:Refresh",
                   curses.color_pair(C_DIM))
    except curses.error:
        pass


# ─────────────────────────────────────────────
#  Colour pairs
# ─────────────────────────────────────────────

C_HEADER  = 1
C_TAB_ACT = 2
C_TAB_IN  = 3
C_VALUE   = 4
C_DIM     = 5
C_STATUS  = 6
C_UPDATED = 7
C_RW      = 8
C_ERROR   = 9
# Graph bar green shades (subtle gradient)
C_GREEN_1 = 10
C_GREEN_2 = 11
C_GREEN_3 = 12
C_GREEN_4 = 13

def init_colours():
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(C_HEADER,  curses.COLOR_CYAN,  -1)
    curses.init_pair(C_TAB_ACT, curses.COLOR_BLACK, curses.COLOR_CYAN)
    curses.init_pair(C_TAB_IN,  curses.COLOR_CYAN,  -1)
    curses.init_pair(C_VALUE,   curses.COLOR_WHITE, -1)
    curses.init_pair(C_DIM,     curses.COLOR_WHITE, -1)
    curses.init_pair(C_STATUS,  curses.COLOR_BLACK, curses.COLOR_CYAN)
    curses.init_pair(C_UPDATED, curses.COLOR_GREEN, -1)
    curses.init_pair(C_RW,      curses.COLOR_GREEN, -1)
    curses.init_pair(C_ERROR,   curses.COLOR_RED,   -1)
    # Graph bar green shades - subtle gradient from dark to light
    curses.init_pair(C_GREEN_1, 28, -1)   # Dark green
    curses.init_pair(C_GREEN_2, 34, -1)   # Medium-dark green
    curses.init_pair(C_GREEN_3, 35, -1)   # Medium green
    curses.init_pair(C_GREEN_4, 40, -1)   # Light green

# ─────────────────────────────────────────────
#  Drawing
# ─────────────────────────────────────────────

def draw_tabs(win, active: int, cols: int):
    win.move(0, 0)
    win.clrtoeol()
    x = 0
    for i, (name, key, _) in enumerate(TABS):
        # renders as e.g. "(L)ive"
        label = f" ({key}){name} "
        attr  = curses.color_pair(C_TAB_ACT) | curses.A_BOLD if i == active \
                else curses.color_pair(C_TAB_IN)
        try:
            win.addstr(0, x, label, attr)
        except curses.error:
            pass
        x += len(label) + 1

def draw_header(win, port: str, cols: int):
    title = f" Victron MPPT  ·  VE.Direct TUI  ·  {port} "
    try:
        win.addstr(1, 0, title.ljust(cols),
                   curses.color_pair(C_HEADER) | curses.A_BOLD)
    except curses.error:
        pass

def draw_status(win, rows: int, cols: int):
    hint = " Q:Quit  Tab/←→:Tabs  R:Refresh  L C H S:Jump  [Settings: type # + Enter to edit] "
    try:
        win.addstr(rows - 1, 0, hint[:cols].ljust(cols),
                   curses.color_pair(C_STATUS))
    except curses.error:
        pass

def draw_live_tab(win, state: State, rows: int, cols: int):
    with state.lock:
        data    = list(state.live_rows)
        updated = state.live_updated

    age     = time.time() - updated if updated else None
    age_str = f"  Last update: {age:.1f}s ago" if age is not None \
              else "  Waiting for data..."
    age_attr = curses.color_pair(C_UPDATED) if age is not None and age < 5 \
               else curses.color_pair(C_ERROR)
    try:
        win.addstr(2, 0, age_str[:cols], age_attr)
    except curses.error:
        pass

    for i, (label, value) in enumerate(data):
        display_row = 3 + i
        if display_row >= rows - 1:
            break
        try:
            win.addstr(display_row, 2,  f"{label:<28}", curses.color_pair(C_DIM))
            win.addstr(display_row, 31, f"{value}",     curses.color_pair(C_VALUE) | curses.A_BOLD)
        except curses.error:
            pass

def draw_hex_tab(win, state: State, addresses: list, rows: int, cols: int):
    with state.lock:
        cache   = dict(state.hex_cache)
        loading = set(state.hex_loading)

    for i, addr in enumerate(addresses):
        display_row = 3 + i
        if display_row >= rows - 1:
            break
        reg = REG_BY_ADDR.get(addr)
        if not reg:
            continue
        if addr in loading:
            value_str = "loading..."
            val_attr  = curses.color_pair(C_DIM)
        elif addr in cache:
            value_str = cache[addr]
            val_attr  = curses.color_pair(C_VALUE) | curses.A_BOLD
        else:
            value_str = "—"
            val_attr  = curses.color_pair(C_DIM)
        try:
            win.addstr(display_row, 2,  f"{reg.name:<30}", curses.color_pair(C_DIM))
            win.addstr(display_row, 33, value_str,         val_attr)
        except curses.error:
            pass

def draw_settings_tab(win, state: State, addresses: list,
                      rows: int, cols: int, cursor: int = 0):
    with state.lock:
        cache   = dict(state.hex_cache)
        loading = set(state.hex_loading)
        log     = list(state.write_log)

    alg = get_current_algorithm(cache)
    system_v = get_system_voltage(cache)
    voltage_factor = system_v // 12

    # ── Algorithm banner ──────────────────────
    if alg is not None:
        abs_v = alg.absorption * voltage_factor
        float_v = alg.float_v * voltage_factor
        eq_v = alg.equalize * voltage_factor if alg.equalize else 0
        if alg.is_lithium:
            banner = f"  ⚡ {system_v}V  |  LiFePO4  |  Abs {abs_v}V  Float {float_v}V"
            ban_attr = curses.color_pair(C_UPDATED) | curses.A_BOLD
        elif alg.is_custom:
            banner = f"  ⚙  {system_v}V  |  Custom / User Defined  |  All parameters user-configurable"
            ban_attr = curses.color_pair(C_DIM)
        else:
            banner = f"  ● {system_v}V  |  Pos {alg.position}: {alg.name}  |  Abs {abs_v}V  Float {float_v}V  Eq {eq_v}V @ {alg.eq_current}%"
            ban_attr = curses.color_pair(C_HEADER)
    else:
        banner = f"  {system_v}V  |  Algorithm: detecting..."
        ban_attr = curses.color_pair(C_DIM)
    try:
        win.addstr(2, 0, banner[:cols].ljust(cols), ban_attr)
    except curses.error:
        pass

    writable = [a for a in addresses
                if a in REG_BY_ADDR and "W" in REG_BY_ADDR[a].rw]
    readonly = [a for a in addresses
                if a in REG_BY_ADDR and "W" not in REG_BY_ADDR[a].rw]

    num = 1
    all_rows = []
    for addr in writable:
        all_rows.append(('rw', num, addr))
        num += 1
    if readonly:
        all_rows.append(('sep', None, None))
        for addr in readonly:
            all_rows.append(('ro', None, addr))

    for i, row in enumerate(all_rows):
        display_row = 4 + i   # row 2 = banner, row 3 = blank gap
        if display_row >= rows - 3:
            break
        kind = row[0]
        if kind == 'sep':
            try:
                win.addstr(display_row, 2, '─' * min(60, cols - 4),
                           curses.color_pair(C_DIM))
            except curses.error:
                pass
            continue

        _, num_label, addr = row
        reg     = REG_BY_ADDR[addr]
        locked  = is_register_locked(addr, alg)
        v_range = get_range_for_register(addr, alg, system_v)
        selected = (kind == 'rw') and (num_label - 1 == cursor)

        if locked:
            # Show actual value even when locked, just with locked styling
            if addr in cache:
                value_str = cache[addr]
                val_attr  = curses.color_pair(C_DIM) | curses.A_DIM
            else:
                value_str = "—"
                val_attr  = curses.color_pair(C_DIM)
        elif addr in loading:
            value_str = "loading..."
            val_attr  = curses.color_pair(C_DIM)
        elif addr in cache:
            # Special display for Battery Type — show algorithm name
            if addr == 0xEDF1:
                value_str = get_battery_type_display(cache)
            else:
                value_str = cache[addr]
            val_attr  = curses.color_pair(C_VALUE) | curses.A_BOLD
            if v_range:
                try:
                    v = float(value_str.split()[0])
                    if v < v_range[0] or v > v_range[1]:
                        val_attr = curses.color_pair(C_ERROR) | curses.A_BOLD
                except Exception:
                    pass
        else:
            value_str = "—"
            val_attr  = curses.color_pair(C_DIM)

        if selected:
            if locked:
                row_attr = curses.color_pair(C_ERROR) | curses.A_BOLD
                val_attr = curses.color_pair(C_ERROR) | curses.A_BOLD
            else:
                row_attr = curses.color_pair(C_TAB_ACT) | curses.A_BOLD
                val_attr = curses.color_pair(C_TAB_ACT) | curses.A_BOLD
        else:
            row_attr = curses.color_pair(C_DIM) if locked \
                       else (curses.color_pair(C_RW) if kind == 'rw'
                             else curses.color_pair(C_DIM))

        range_hint = f"  [{v_range[0]}–{v_range[1]}V]" if v_range and not locked else ""

        try:
            if kind == 'rw':
                num_str = "   " if locked else f"{num_label:>2}."
                win.addstr(display_row, 2,  num_str,             row_attr)
                win.addstr(display_row, 6,  f"{reg.name:<32}",  row_attr)
            else:
                win.addstr(display_row, 2,  "   ",               curses.color_pair(C_DIM))
                win.addstr(display_row, 6,  f"{reg.name:<32}",  curses.color_pair(C_DIM))
            win.addstr(display_row, 39, value_str, val_attr)
            if range_hint:
                win.addstr(display_row, 39 + len(value_str) + 1,
                           range_hint, curses.color_pair(C_DIM))
        except curses.error:
            pass

    if log:
        try:
            win.addstr(rows - 3, 2,
                       f"Last: {log[-1]}"[:cols - 3],
                       curses.color_pair(C_UPDATED))
        except curses.error:
            pass
    try:
        win.addstr(rows - 2, 2,
                   "↑↓:Select  Enter:Edit  or type a number  |  A:Algorithm selector",
                   curses.color_pair(C_DIM))
    except curses.error:
        pass

# ─────────────────────────────────────────────
#  ESC-aware input helper
# ─────────────────────────────────────────────

def raw_input(prompt: str) -> Optional[str]:
    """
    Like input() but returns None if ESC is pressed.
    Uses termios/tty for char-by-char reading on macOS/Linux.
    """
    import sys, tty, termios
    sys.stdout.write(prompt)
    sys.stdout.flush()
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    buf = []
    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.read(1)
            if ch == '\x1b':          # ESC
                sys.stdout.write('\r\n')
                return None
            elif ch in ('\r', '\n'):  # Enter
                sys.stdout.write('\r\n')
                return ''.join(buf)
            elif ch in ('\x7f', '\x08'):  # Backspace
                if buf:
                    buf.pop()
                    sys.stdout.write('\b \b')
                    sys.stdout.flush()
            elif ch == '\x03':        # Ctrl-C
                raise KeyboardInterrupt
            elif ch >= ' ':
                buf.append(ch)
                sys.stdout.write(ch)
                sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

# ─────────────────────────────────────────────
#  Edit prompt
# ─────────────────────────────────────────────

def edit_register(stdscr, worker: SerialWorker, state: State,
                  addresses: list, reg_num: int):
    """Edit a register with chemistry-aware validation."""
    writable = [a for a in addresses
                if a in REG_BY_ADDR and "W" in REG_BY_ADDR[a].rw]
    if reg_num < 1 or reg_num > len(writable):
        return

    addr = writable[reg_num - 1]
    reg  = REG_BY_ADDR[addr]

    with state.lock:
        cache = dict(state.hex_cache)
    alg     = get_current_algorithm(cache)
    system_v = get_system_voltage(cache)
    locked  = is_register_locked(addr, alg)
    v_range = get_range_for_register(addr, alg, system_v)

    curses.endwin()

    if locked:
        print(f"\n\033[1;31m─── Blocked: {reg.name} ──────────────────────────────────\033[0m")
        if alg and alg.is_lithium:
            print(f"  \033[31mThis register is N/A for LiFePO4 chemistry.\033[0m")
            print(f"  Equalisation and temperature compensation must not be used")
            print(f"  with lithium batteries. This block protects your battery.")
        else:
            print(f"  \033[31mThis register is locked for the current algorithm.\033[0m")
        print()
        time.sleep(2.0)
        return

    print(f"\n\033[1;36m─── Edit: {reg.name} ─────────────────────────────────\033[0m")
    print(f"  {reg.desc}")
    if reg.unit:
        print(f"  Unit: \033[36m{reg.unit}\033[0m   Scale: {reg.scale}")
    if v_range:
        print(f"  \033[33m⚠  Safe range for {alg.name if alg else 'current chemistry'}: "
              f"{v_range[0]}–{v_range[1]} {reg.unit}\033[0m")
    if reg.choices:
        print("  Valid values:")
        for k, v in reg.choices.items():
            print(f"    {k:>4}  =  {v}")

    worker.ping()
    val_bytes = worker.get_register(reg.address)
    if val_bytes is not None:
        print(f"  Current: \033[1m{decode_value(reg, val_bytes)}\033[0m")
    else:
        print("  Could not read current value.")

    inp = raw_input(f"\n  New value in {reg.unit or 'raw units'} (ESC to cancel): ")
    if inp is None or not inp.strip():
        print("  Cancelled.")
        time.sleep(0.8)
        return

    try:
        new_value = float(inp.strip())
    except ValueError:
        print("  Invalid value. Cancelled.")
        time.sleep(0.8)
        return

    # Range validation
    if v_range and (new_value < v_range[0] or new_value > v_range[1]):
        print(f"\n  \033[1;31m⚠  WARNING: {new_value} {reg.unit} is outside the safe range "
              f"({v_range[0]}–{v_range[1]} {reg.unit})\033[0m")
        print(f"  Values outside this range may damage your battery.")
        confirm_range = raw_input("  Type OVERRIDE to proceed anyway, ESC or any other key cancels: ")
        if confirm_range is None or confirm_range.strip().upper() != 'OVERRIDE':
            print("  Cancelled.")
            time.sleep(0.8)
            return

    raw_value = scaled_to_raw(reg, new_value)
    print(f"\n  Will write: \033[1m{new_value} {reg.unit}\033[0m"
          f"  →  raw {raw_value} (0x{raw_value & 0xFFFF:04X})")
    confirm = raw_input("  Type Y to confirm, any other key cancels: ")
    if confirm is None or confirm.strip().upper() != 'Y':
        print("  Cancelled.")
        time.sleep(0.8)
        return

    print(f"  Writing ...", end=" ", flush=True)
    worker.ping()
    ok, flags = worker.set_register(reg.address, raw_value, size=reg.size)

    if ok:
        print("\033[32mOK\033[0m")
        with state.lock:
            state.write_log.append(
                f"✓  {reg.name} = {new_value} {reg.unit}  (raw {raw_value})")
            state.hex_cache[reg.address] = f"{new_value} {reg.unit}".strip()

        save = raw_input("  Save to NVM? (Y to confirm, any other key skips): ")
        if save and save.strip().upper() == 'Y':
            print("  Saving ...", end=" ", flush=True)
            worker.ping()
            if worker.nvm_save():
                print("\033[32mOK\033[0m")
                with state.lock:
                    state.write_log.append("     → Saved to NVM")
            else:
                print("\033[31mFailed\033[0m")
                with state.lock:
                    state.write_log.append("     → NVM save failed")
        else:
            print("  Skipped NVM save.")
    else:
        msg = {0x02:"read-only", 0x04:"parameter error"}.get(flags, "failed")
        print(f"\033[31mFailed ({msg})\033[0m")
        with state.lock:
            state.write_log.append(f"✗  {reg.name} — {msg}")

    time.sleep(1.5)


def algorithm_selector(stdscr, worker: SerialWorker, state: State):
    """Apply a preset charging algorithm to the controller."""
    curses.endwin()
    
    # Get current system voltage
    with state.lock:
        cache = dict(state.hex_cache)
    system_v = get_system_voltage(cache)
    
    # Ask for voltage selection first
    print(f"\n\033[1;36m─── Algorithm Selector ───────────────────────────────────\033[0m")
    print(f"  Current System Voltage: {system_v}V")
    print(f"  Select system voltage: 1 = 12V, 2 = 24V")
    inp = raw_input("  (or press Enter to keep current): ")
    if inp and inp.strip() in ('1', '2'):
        new_v = 12 if inp.strip() == '1' else 24
        if new_v != system_v:
            # Write new voltage setting
            REG_BATTERY_V = Register(0xEDEA, "Battery V Setting", "", "", 1, "RW", "d", 1)
            worker.ping()
            ok, flags = worker.set_register(REG_BATTERY_V.address, new_v, size=1)
            if ok:
                print(f"  \033[32mSet to {new_v}V\033[0m")
                system_v = new_v
            else:
                print(f"  \033[31mFailed to set voltage\033[0m")
                time.sleep(1.5)
                return
    print()
    print(f"  {'#':<4} {'Name':<38} {'Abs':>6} {'Float':>6} {'Eq':>7} {'Eq%':>4} {'Temp':>6}")
    print(f"  {'─'*74}")
    for alg in ALGORITHMS:
        if alg.is_custom:
            print(f"  {alg.position:<4} \033[2m{alg.name:<38}\033[0m  (manual edit only)")
        else:
            # Scale voltages for system voltage
            abs_v = alg.absorption * (system_v // 12)
            float_v = alg.float_v * (system_v // 12)
            eq_v = alg.equalize * (system_v // 12) if alg.equalize else 0
            eq_str  = "  n/a" if eq_v == 0  else f"{eq_v:>5}V"
            eqp_str = " n/a" if alg.eq_current == 0 else f"{alg.eq_current:>3}%"
            tc_str  = "   0" if alg.temp_comp == 0   else f"{alg.temp_comp:>4}"
            marker  = " \033[33m← LiFePO4\033[0m" if alg.is_lithium else ""
            print(f"  {alg.position:<4} {alg.name:<38} "
                  f"{abs_v:>5}V {float_v:>5}V "
                  f"{eq_str} {eqp_str} {tc_str}{marker}")
    print()
    print(f"  \033[33mNote: This writes absorption, float, temp compensation and\033[0m")
    print(f"  \033[33mequalise voltage to the controller as a starting point.\033[0m")
    print(f"  \033[33mBattery type is set to match the selected preset.\033[0m")
    print()

    inp = raw_input("  Select algorithm 0–8 (ESC to cancel): ")
    if inp is None or not inp.strip():
        print("  Cancelled.")
        time.sleep(0.8)
        return

    try:
        pos = int(inp.strip())
        alg = ALG_BY_POS[pos]
    except (ValueError, KeyError):
        print("  Invalid selection.")
        time.sleep(0.8)
        return

    if alg.is_custom:
        print("  Custom mode selected — no values written. Edit registers individually.")
        time.sleep(1.5)
        return

    # Scale voltages for system voltage
    voltage_factor = system_v // 12
    abs_v = alg.absorption * voltage_factor
    float_v = alg.float_v * voltage_factor
    eq_v = alg.equalize * voltage_factor if alg.equalize else 0
    
    print(f"\n  Will apply: \033[1m{alg.name}\033[0m @ {system_v}V")
    print(f"    Absorption voltage : {abs_v} V")
    print(f"    Float voltage      : {float_v} V")
    if eq_v > 0:
        print(f"    Equalise voltage   : {eq_v} V")
    else:
        print(f"    Equalise voltage   : n/a (not written)")
    print(f"    Temp compensation  : {alg.temp_comp} mV/°C")
    print(f"    Battery type       : {alg.name} (preset {alg.position})")
    print()

    confirm = raw_input("  Type Y to apply this algorithm, any other key cancels: ")
    if confirm is None or confirm.strip().upper() != 'Y':
        print("  Cancelled.")
        time.sleep(0.8)
        return

    REG_BATTERY_TYPE  = Register(0xEDF1, "Battery Type", "", "", 1, "RW", "d", 1)
    REG_BATTERY_V     = Register(0xEDEA, "Battery V Setting", "", "", 1, "RW", "d", 1)
    REG_ABSORPTION    = Register(0xEDF7, "Absorption Voltage", "", "", 0.01, "RW", "d", 2)
    REG_FLOAT         = Register(0xEDF6, "Float Voltage", "", "", 0.01, "RW", "d", 2)
    REG_EQUALIZE      = Register(0xEDF4, "Equalise Voltage", "", "", 0.01, "RW", "d", 2)
    REG_TEMP_COMP     = Register(0xEDF2, "Temp Compensation", "", "", 0.01, "RW", "d", 2)

    # Set battery type to match algorithm position (not 255 = user defined)
    # Scale voltages for system voltage
    voltage_factor = system_v // 12
    abs_raw = round(abs_v / 0.01)
    float_raw = round(float_v / 0.01)
    eq_raw = round(eq_v / 0.01) if eq_v > 0 else 0
    
    writes = [
        (REG_BATTERY_V,     system_v, f"System Voltage = {system_v}V"),
        # Register value = algorithm position + 1 (position 0 = register 1, etc.)
        (REG_BATTERY_TYPE, alg.position + 1, f"Battery Type = {alg.name}"),
        (REG_ABSORPTION,   abs_raw, f"Absorption = {abs_v}V"),
        (REG_FLOAT,        float_raw,    f"Float = {float_v}V"),
        (REG_TEMP_COMP,    round(alg.temp_comp / 0.01) & 0xFFFF, f"Temp comp = {alg.temp_comp} mV/°C"),
    ]
    if eq_raw > 0:
        writes.append((REG_EQUALIZE, eq_raw, f"Equalise = {eq_v}V"))

    all_ok = True
    for reg_obj, raw_val, desc in writes:
        print(f"  Writing {desc} ...", end=" ", flush=True)
        worker.ping()
        ok, flags = worker.set_register(reg_obj.address, raw_val, size=reg_obj.size)
        if ok:
            print("\033[32mOK\033[0m")
            with state.lock:
                state.hex_cache[reg_obj.address] = desc.split("=")[1].strip()
        else:
            msg = {0x02:"read-only", 0x04:"param error"}.get(flags, "failed")
            print(f"\033[31mFailed ({msg})\033[0m")
            all_ok = False

    if all_ok:
        print()
        save = raw_input("  All writes OK. Save to NVM? (Y to confirm): ")
        if save and save.strip().upper() == 'Y':
            print("  Saving ...", end=" ", flush=True)
            worker.ping()
            if worker.nvm_save():
                print("\033[32mOK\033[0m")
                with state.lock:
                    state.write_log.append(f"✓  Applied algorithm: {alg.name}")
                    state.write_log.append("     → Saved to NVM")
            else:
                print("\033[31mFailed\033[0m")
        else:
            print("  Skipped NVM save (settings active until power cycle).")
    else:
        print("\n  \033[31mOne or more writes failed. Check settings manually.\033[0m")

    time.sleep(2.0)

# ─────────────────────────────────────────────
#  Main TUI loop
# ─────────────────────────────────────────────

def tui_main(stdscr, worker: SerialWorker, port: str):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    init_colours()

    state             = State()
    active_tab        = 0
    stop_event        = threading.Event()
    fetched           = set()
    history_graph_mode = False   # True = show graph on History tab

    # TEXT live reader
    t_live = threading.Thread(target=live_reader_thread,
                              args=(worker, state, stop_event), daemon=True)
    t_live.start()

    def fetch_tab(idx, force=False):
        addrs = TABS[idx][2]
        if not addrs:
            return
        if idx in fetched and not force:
            return
        fetched.add(idx)
        if force:
            with state.lock:
                for a in addrs:
                    state.hex_cache.pop(a, None)
                    state.hex_loading.add(a)
                    state.hex_retries.pop(a, None)  # reset retry count
        t = threading.Thread(target=hex_fetch_thread,
                             args=(worker, state, addrs, stop_event),
                             daemon=True)
        t.start()

    # pre-fetch all HEX tabs
    all_hex_addrs = []
    for idx in range(1, len(TABS)):
        fetch_tab(idx)
        all_hex_addrs.extend(TABS[idx][2])

    # auto-refetch anything that comes back as "—"
    threading.Thread(target=auto_refetch_thread,
                     args=(worker, state, all_hex_addrs, stop_event),
                     daemon=True).start()

    last_draw       = 0.0
    num_buf         = ""   # accumulates digit keypresses on settings tab
    settings_cursor = 0    # which writable register is highlighted

    try:
        while True:
            rows, cols = stdscr.getmaxyx()

            try:
                key = stdscr.getch()
            except Exception:
                key = -1

            if key in (ord('q'), ord('Q')):
                break
            elif key == curses.KEY_RIGHT or key == ord('\t'):
                active_tab = (active_tab + 1) % len(TABS)
                history_graph_mode = False
                num_buf = ""
            elif key == curses.KEY_LEFT:
                active_tab = (active_tab - 1) % len(TABS)
                history_graph_mode = False
                num_buf = ""
            elif key == curses.KEY_DOWN and active_tab == 3:
                tab_addrs   = TABS[3][2]
                writable_ct = sum(1 for a in tab_addrs
                                  if a in REG_BY_ADDR and "W" in REG_BY_ADDR[a].rw)
                settings_cursor = min(settings_cursor + 1, writable_ct - 1)
                num_buf = ""
            elif key == curses.KEY_UP and active_tab == 3:
                settings_cursor = max(settings_cursor - 1, 0)
                num_buf = ""
            elif key != -1:
                ch = chr(key).lower() if key < 256 else ''
                if ch in TAB_KEYS:
                    active_tab = TAB_KEYS[ch]
                    history_graph_mode = False
                    num_buf = ""
                elif ch == 'r' and active_tab != 0:
                    if active_tab == 2 and history_graph_mode:
                        # Refresh graph data
                        with state.lock:
                            state.graph_data    = []
                            state.graph_loading = True
                        threading.Thread(target=graph_fetch_thread,
                                         args=(worker, state, stop_event),
                                         daemon=True).start()
                    else:
                        fetch_tab(active_tab, force=True)
                    num_buf = ""
                elif ch == 'g' and active_tab == 2:
                    # Switch History tab to graph view, fetch if needed
                    history_graph_mode = True
                    if not state.graph_data and not state.graph_loading:
                        with state.lock:
                            state.graph_loading = True
                        threading.Thread(target=graph_fetch_thread,
                                         args=(worker, state, stop_event),
                                         daemon=True).start()
                elif ch == 'b' and active_tab == 2 and history_graph_mode:
                    history_graph_mode = False
                elif active_tab == 3:  # Settings tab
                    if key in (curses.KEY_BACKSPACE, 127, 8):
                        num_buf = num_buf[:-1]
                    elif ch == 'a':
                        # Algorithm selector
                        num_buf = ""
                        stop_event.set()
                        t_live.join(timeout=2)
                        algorithm_selector(stdscr, worker, state)
                        # refresh settings cache after algorithm write
                        fetch_tab(3, force=True)
                        stop_event.clear()
                        t_live = threading.Thread(
                            target=live_reader_thread,
                            args=(worker, state, stop_event), daemon=True)
                        t_live.start()
                        stdscr = curses.initscr()
                        curses.curs_set(0)
                        stdscr.nodelay(True)
                        stdscr.keypad(True)
                        init_colours()
                    elif ch.isdigit():
                        num_buf += ch
                        n = int(num_buf)
                        tab_addrs   = TABS[3][2]
                        writable_ct = sum(1 for a in tab_addrs
                                          if a in REG_BY_ADDR and "W" in REG_BY_ADDR[a].rw)
                        if 1 <= n <= writable_ct:
                            settings_cursor = n - 1
                    elif key in (ord('\n'), ord('\r')):
                        reg_num = int(num_buf) if num_buf else settings_cursor + 1
                        num_buf = ""
                        tab_addrs = TABS[3][2]
                        stop_event.set()
                        t_live.join(timeout=2)
                        edit_register(stdscr, worker, state,
                                      tab_addrs, reg_num)
                        stop_event.clear()
                        t_live = threading.Thread(
                            target=live_reader_thread,
                            args=(worker, state, stop_event), daemon=True)
                        t_live.start()
                        stdscr = curses.initscr()
                        curses.curs_set(0)
                        stdscr.nodelay(True)
                        stdscr.keypad(True)
                        init_colours()
                    elif key == 27:   # ESC clears number buffer
                        num_buf = ""

            now = time.time()
            if now - last_draw < 0.1:
                time.sleep(0.02)
                continue
            last_draw = now

            stdscr.erase()
            draw_tabs(stdscr, active_tab, cols)
            draw_header(stdscr, port, cols)

            tab_name, tab_key, tab_addrs = TABS[active_tab]
            if active_tab == 0:
                draw_live_tab(stdscr, state, rows, cols)
            elif active_tab == 2:
                if history_graph_mode:
                    draw_graph_tab(stdscr, state, rows, cols, start_row=2)
                else:
                    draw_hex_tab(stdscr, state, tab_addrs, rows, cols)
                    try:
                        stdscr.addstr(rows - 2, 2, "G:Graph view",
                                      curses.color_pair(C_DIM))
                    except curses.error:
                        pass
            elif active_tab == 3:
                draw_settings_tab(stdscr, state, tab_addrs, rows, cols,
                                  settings_cursor)
                if num_buf:
                    try:
                        stdscr.addstr(rows - 2, cols - len(num_buf) - 12,
                                      f"  Edit #: {num_buf}█",
                                      curses.color_pair(C_RW) | curses.A_BOLD)
                    except curses.error:
                        pass
            else:
                draw_hex_tab(stdscr, state, tab_addrs, rows, cols)

            draw_status(stdscr, rows, cols)
            stdscr.refresh()

    finally:
        stop_event.set()

# ─────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────

def find_port() -> Optional[str]:
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if "ve direct" in desc or (p.vid == 0x0403 and p.pid == 0x6001):
            return p.device
        if "usbserial" in p.device.lower():
            return p.device
    return None

def main():
    parser = argparse.ArgumentParser(
        description="VE.Direct TUI — Victron MPPT terminal interface")
    parser.add_argument("port", nargs="?",
                        help="Serial port (auto-detected if omitted)")
    args = parser.parse_args()

    port = args.port or find_port()
    if not port:
        print("No VE.Direct device found. Specify port manually:")
        print("  python3 vedirect_tui.py /dev/tty.usbserial-XXXXX")
        sys.exit(1)

    print(f"Connecting to {port} ...", end=" ", flush=True)
    try:
        ser = serial.Serial(port=port, baudrate=115200,
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            timeout=0.1)
    except serial.SerialException as e:
        print(f"Failed\n  {e}")
        sys.exit(1)
    print("OK")

    worker = SerialWorker(ser)
    time.sleep(0.5)

    try:
        curses.wrapper(tui_main, worker, port)
    except KeyboardInterrupt:
        pass
    finally:
        worker.stop()
        ser.close()
    print("Bye.")

if __name__ == "__main__":
    main()
