"""
Victron BlueSolar OLED Display — MicroPython for Raspberry Pi Pico 2
Auto-cycles through 5 screens every 7 seconds.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
WIRING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
VE.Direct JST 4-pin (pin 1 nearest locking tab):

  VE.Direct Pin 1 (GND)  ──── Pico GND        (pin 38)
  VE.Direct Pin 2 (RX)   ──── not connected
  VE.Direct Pin 3 (TX)   ──── Pico GP9         (pin 12) [UART1 RX]
  VE.Direct Pin 4 (+5V)  ──── not connected

SSD1306 OLED (I2C, 128x32):

  OLED VCC  ──── Pico 3.3V      (pin 36)
  OLED GND  ──── Pico GND       (pin 38)
  OLED SDA  ──── Pico GP4       (pin 6)   [I2C0 SDA]
  OLED SCL  ──── Pico GP5       (pin 7)   [I2C0 SCL]

POWER:
  Power the Pico via USB from a power bank or USB supply.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

from machine import UART, I2C, Pin
import ssd1306
import utime

# ── UART1 (VE.Direct) ─────────────────────────────────────────────────────────
uart = UART(1, baudrate=19200, rx=Pin(9))

# ── OLED (I2C0) ───────────────────────────────────────────────────────────────
i2c  = I2C(0, scl=Pin(5), sda=Pin(4))
oled = ssd1306.SSD1306_I2C(128, 32, i2c)

# ── Voltage calibration offset (mV) ──────────────────────────────────────────
# Increase this value if the SOC reads lower than expected.
# e.g. 200 means we add 0.2V to the voltage before estimating SOC.
V_CAL_MV = 200

# ── Charge state lookup ───────────────────────────────────────────────────────
CS_NAMES = {
    0:   'Off',
    2:   'Fault',
    3:   'Bulk',
    4:   'Absorb',
    5:   'Float',
    7:   'Equal',
    245: 'Start',
}

# ── LiFePO4 voltage to SOC lookup (12V, resting voltage) ─────────────────────
LIFEPO4_SOC = [
    (14400, 100),
    (13600,  99),
    (13400,  90),
    (13200,  70),
    (13100,  40),
    (13000,  30),
    (12800,  20),
    (12000,  10),
    (10000,   0),
]

def estimate_soc(mv):
    try:
        mv = int(mv) + V_CAL_MV
    except:
        return None
    if mv >= LIFEPO4_SOC[0][0]:  return 100
    if mv <= LIFEPO4_SOC[-1][0]: return 0
    for i in range(len(LIFEPO4_SOC) - 1):
        v_hi, s_hi = LIFEPO4_SOC[i]
        v_lo, s_lo = LIFEPO4_SOC[i + 1]
        if v_lo <= mv <= v_hi:
            return int(s_lo + (mv - v_lo) / (v_hi - v_lo) * (s_hi - s_lo))
    return None

# ── Shared data ───────────────────────────────────────────────────────────────
data = {
    'V':    None,  # battery voltage (mV)
    'I':    None,  # battery current (mA)
    'VPV':  None,  # panel voltage (mV)
    'PPV':  None,  # panel power (W)
    'CS':   None,  # charge state
    'LOAD': None,  # load on/off
    'H20':  None,  # yield today (0.01 kWh)
    'H19':  None,  # yield total (0.01 kWh)
}

# ── VE.Direct parser ──────────────────────────────────────────────────────────
class VEDirectParser:
    def __init__(self):
        self.block    = {}
        self.line_buf = ''

    def feed_byte(self, b):
        char = chr(b)
        if char == '\n':
            self._process(self.line_buf)
            self.line_buf = ''
        else:
            self.line_buf += char

    def _process(self, line):
        line = line.strip('\r')
        if not line or '\t' not in line:
            return
        key, _, val = line.partition('\t')
        if key == 'Checksum':
            if self.block:
                for k in data:
                    if k in self.block:
                        data[k] = self.block[k]
            self.block = {}
        else:
            try:
                self.block[key] = int(val)
            except:
                self.block[key] = val

# ── Formatting helpers ────────────────────────────────────────────────────────
def fmt_v(val):
    try:    return '{:.2f}V'.format(int(val) / 1000)
    except: return '---'

def fmt_a(val):
    try:    return '{:+.2f}A'.format(int(val) / 1000)
    except: return '---'

def fmt_w(val):
    try:    return '{}W'.format(int(val))
    except: return '---'

def fmt_yield_wh(val):
    try:    return '{}Wh'.format(int(val) * 10)
    except: return '---'

def fmt_total(val):
    try:    return '{:.1f}'.format(int(val) / 100)
    except: return '---'

def fmt_cs(val):
    try:    return CS_NAMES.get(int(val), 'CS:{}'.format(val))
    except: return '---'

# ── Battery icon ──────────────────────────────────────────────────────────────
def draw_battery_screen(pct):
    bat_w, bat_h = 76, 22
    nub_w, nub_h = 4, 10
    gap          = 5
    pct_str      = '{}%'.format(pct)
    txt_w        = len(pct_str) * 8
    total_w      = bat_w + nub_w + gap + txt_w
    x  = (128 - total_w) // 2
    y  = (32  - bat_h)   // 2
    oled.rect(x, y, bat_w, bat_h, 1)
    oled.fill_rect(x + bat_w, y + (bat_h - nub_h) // 2, nub_w, nub_h, 1)
    fill_w = max(0, (bat_w - 4) * pct // 100)
    if fill_w > 0:
        oled.fill_rect(x + 2, y + 2, fill_w, bat_h - 4, 1)
    oled.text(pct_str, x + bat_w + nub_w + gap, y + (bat_h - 8) // 2)

# ── Screen renderers ──────────────────────────────────────────────────────────
def draw_screen(screen_idx):
    oled.fill(0)

    if screen_idx == 0:
        oled.text('BATTERY', 0, 0)
        oled.text(fmt_v(data['V']), 0, 16)
        oled.text('CURR', 72, 0)
        oled.text(fmt_a(data['I']), 72, 16)

    elif screen_idx == 1:
        oled.text('PANEL', 0, 0)
        oled.text(fmt_v(data['VPV']), 0, 16)
        oled.text('POWER', 72, 0)
        oled.text(fmt_w(data['PPV']), 72, 16)

    elif screen_idx == 2:
        oled.text('STATE', 0, 0)
        oled.text(fmt_cs(data['CS']), 0, 16)
        oled.text('LOAD', 72, 0)
        oled.text(str(data['LOAD']) if data['LOAD'] else '---', 72, 16)

    elif screen_idx == 3:
        oled.text('TODAY', 0, 0)
        oled.text(fmt_yield_wh(data['H20']), 0, 16)
        oled.text('TOT kWh', 72, 0)
        oled.text(fmt_total(data['H19']), 72, 16)

    elif screen_idx == 4:
        soc = estimate_soc(data['V'])
        draw_battery_screen(soc if soc is not None else 0)

    oled.show()

# ── Main loop ─────────────────────────────────────────────────────────────────
parser       = VEDirectParser()
screen_idx   = 0
NUM_SCREENS  = 5
CYCLE_MS     = 5000
last_cycle   = utime.ticks_ms()

draw_screen(screen_idx)

while True:
    while uart.any():
        b = uart.read(1)
        if b:
            parser.feed_byte(b[0])

    now = utime.ticks_ms()
    if utime.ticks_diff(now, last_cycle) >= CYCLE_MS:
        screen_idx = (screen_idx + 1) % NUM_SCREENS
        last_cycle = now
        draw_screen(screen_idx)

    utime.sleep_ms(10)
