"""
Victron BlueSolar OLED Display — MicroPython for Lolin D1 Mini
Auto-cycles through 4 screens every 2 seconds.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
WIRING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
VE.Direct JST 4-pin (pin 1 nearest locking tab):

  VE.Direct Pin 1 (GND)  ──── D1 Mini GND
  VE.Direct Pin 2 (RX)   ──── not connected (listen only)
  VE.Direct Pin 3 (TX)   ──── D1 Mini RX (GPIO3)  [direct, no LED]
  VE.Direct Pin 4 (+5V)  ──── D1 Mini 5V  (to power the board)

SSD1306 OLED (I2C, 128x32):

  OLED VCC  ──── D1 Mini 3.3V
  OLED GND  ──── D1 Mini GND
  OLED SDA  ──── D1 Mini D2 (GPIO4)
  OLED SCL  ──── D1 Mini D1 (GPIO5)

NOTE: USB must be disconnected when running from VE.Direct 5V.
      UART0 (GPIO3) is shared with USB and will not receive
      VE.Direct data while USB is connected.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

from machine import UART, I2C, Pin
import network
import ssd1306
import utime

# ── Disable WiFi ──────────────────────────────────────────────────────────────
network.WLAN(network.STA_IF).active(False)
network.WLAN(network.AP_IF).active(False)

# ── UART (VE.Direct) ──────────────────────────────────────────────────────────
# UART0 RX = GPIO3 (pin labelled RX on D1 Mini)
# USB must be disconnected for this to work
uart = UART(0, baudrate=115200, rxbuf=256)

# ── OLED (I2C) ────────────────────────────────────────────────────────────────
i2c  = I2C(scl=Pin(5), sda=Pin(4))   # D1=GPIO5, D2=GPIO4
oled = ssd1306.SSD1306_I2C(128, 32, i2c)

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

def fmt_yield(val):
    try:    return '{:.2f}kWh'.format(int(val) / 100)
    except: return '---'

def fmt_total(val):
    try:    return '{:.1f}'.format(int(val) / 100)
    except: return '---'

def fmt_cs(val):
    try:    return CS_NAMES.get(int(val), 'CS:{}'.format(val))
    except: return '---'

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
        oled.text(fmt_yield(data['H20']), 0, 16)
        oled.text('TOTAL', 72, 0)
        oled.text(fmt_total(data['H19']), 72, 16)

    oled.show()

# ── Main loop ─────────────────────────────────────────────────────────────────
parser     = VEDirectParser()
screen_idx = 0
NUM_SCREENS  = 4
CYCLE_MS     = 2000
last_cycle   = utime.ticks_ms()

while True:
    # Drain all available UART bytes
    while uart.any():
        b = uart.read(1)
        if b:
            parser.feed_byte(b[0])

    # Cycle screen on timer
    now = utime.ticks_ms()
    if utime.ticks_diff(now, last_cycle) >= CYCLE_MS:
        screen_idx = (screen_idx + 1) % NUM_SCREENS
        last_cycle = now
        draw_screen(screen_idx)

    utime.sleep_ms(10)
