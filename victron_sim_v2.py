#!/usr/bin/env python3
"""
Victron BlueSolar OLED Simulator v2
Uses the karioja/vedirect library for parsing.
Screens auto-cycle every 4 seconds — no button required.

Install dependencies:
  pip install vedirect pygame

Usage:
  python3 victron_sim_v2.py --port /dev/tty.usbserial-XXXX   # real hardware
  python3 victron_sim_v2.py --demo                             # simulated data

Screens (auto-cycling every 4s):
  1. Battery voltage + current
  2. Solar panel voltage + power
  3. Charge state + load on/off
  4. yield today + yield total
"""

import argparse
import threading
import time
import sys
import pygame
import vedirect

# ── Dimensions ───────────────────────────────────────────────────────────────
OLED_W, OLED_H  = 128, 32
SCALE            = 4
WIN_W            = OLED_W * SCALE
WIN_H            = OLED_H * SCALE + 40   # just enough for screen label

CYCLE_SECONDS    = 2

# ── Colours ──────────────────────────────────────────────────────────────────
BLACK  = (0,   0,   0)
WHITE  = (255, 255, 255)
YELLOW = (255, 220,  50)
GRAY   = (120, 120, 120)
DKGRAY = (30,   30,  30)

# ── Charge state lookup ──────────────────────────────────────────────────────
CS_NAMES = {
    0:   'Off',
    2:   'Fault',
    3:   'Bulk',
    4:   'Absorb',
    5:   'Float',
    7:   'Equal',
    245: 'Start',
}

# ── Shared state ─────────────────────────────────────────────────────────────
data = {
    'V':   None, #battery voltage
    'I':   None, #battery current
    'VPV': None, #panel voltage
    'PPV': None, #panel power
    'CS':  None, #charge state
    'LOAD':  None, #load on/off
    'H20': None, #yield today
    'H19': None, #yield total
    
}
data_lock = threading.Lock()

# ── VE.Direct callback ───────────────────────────────────────────────────────
def on_vedirect_packet(packet):
    with data_lock:
        for key in data:
            if key in packet:
                data[key] = packet[key]

# ── Serial reader thread ─────────────────────────────────────────────────────
def vedirect_thread(port):
    try:
        from vedirect import Vedirect
    except ImportError:
        print("[error] vedirect not found. Run: pip install vedirect")
        sys.exit(1)
    try:
        ve = Vedirect(port, 19200)
        print(f"[serial] connected to {port}")
        ve.read_data_callback(on_vedirect_packet)
    except Exception as e:
        print(f"[serial] ERROR: {e}")

# ── Demo thread ──────────────────────────────────────────────────────────────
def demo_thread():
    import math
    t = 0
    while True:
        on_vedirect_packet({
            'V':   int(12800 + 400 * math.sin(t * 0.05)),
            'I':   int(1500 + 500 * math.sin(t * 0.07)),
            'VPV': int(18000 + 2000 * math.sin(t * 0.03)),
            'PPV': int(max(0, 22 + 18 * math.sin(t * 0.04))),
            'CS':  [3, 4, 5][t // 20 % 3],
            'H19': int(120 + t // 5),
            'H20': int(120 + t // 5),
            'LOAD': ['ON', 'OFF'][t // 20 % 2],
        })
        t += 1
        time.sleep(0.5)

# ── Formatting helpers ────────────────────────────────────────────────────────
def fmt_v(val):
    try:    return f"{int(val)/1000:.2f}V"
    except: return '---'

def fmt_a(val):
    try:    return f"{int(val)/1000:+.2f}A"
    except: return '---'

def fmt_w(val):
    try:    return f"{int(val)}W"
    except: return '---'

def fmt_yield(val):
    try:    return f"{int(val)/100:.2f}kWh"
    except: return '---'

def fmt_total(val):
    try:    return f"{int(val)/100:.1f}"
    except: return '---'

def fmt_cs(val):
    try:    return CS_NAMES.get(int(val), f'CS:{val}')
    except: return '---'

# ── OLED screen renderers ─────────────────────────────────────────────────────
def draw_screen(surf, font_lg, font_sm, screen_idx, d):
    surf.fill(BLACK)

    def label(text, x, y):
        surf.blit(font_sm.render(text, True, GRAY), (x, y))

    def value(text, x, y):
        surf.blit(font_lg.render(text, True, WHITE), (x, y))

    if screen_idx == 0:
        label("BATTERY",  0, 0);  value(fmt_v(d['V']),  0, 10)
        label("CURRENT", 72, 0);  value(fmt_a(d['I']), 72, 10)

    elif screen_idx == 1:
        label("PANEL",   0, 0);  value(fmt_v(d['VPV']),  0, 10)
        label("POWER",  80, 0);  value(fmt_w(d['PPV']), 80, 10)

    elif screen_idx == 2:
        label("STATE",   0, 0);  value(fmt_cs(d['CS']),     0, 10)
        #label("TODAY",  80, 0);  value(fmt_yield(d['H20']), 80, 10)
        label("LOAD",  80, 0);  value(d['LOAD'], 80, 10)

    elif screen_idx == 3:
        label("TODAY",  0, 0);  value(fmt_yield(d['H20']), 0, 10)
        label("TOTAL(kWh)",  76, 0);  value(fmt_total(d['H19']), 76, 10)

# ── Progress bar (shows time until next screen) ───────────────────────────────
def draw_progress(screen, elapsed, total, y):
    bar_w = int(WIN_W * min(elapsed / total, 1.0))
    pygame.draw.rect(screen, (50, 50, 70), (0, y, WIN_W, 3))
    pygame.draw.rect(screen, YELLOW,       (0, y, bar_w, 3))

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=None,
                        help='Serial port e.g. /dev/tty.usbserial-XXXX')
    parser.add_argument('--demo', action='store_true',
                        help='Run with simulated data (no hardware needed)')
    args = parser.parse_args()

    if args.demo or args.port is None:
        if not args.demo:
            print("[info] No --port given, running in demo mode.")
        threading.Thread(target=demo_thread, daemon=True).start()
    else:
        threading.Thread(target=vedirect_thread, args=(args.port,), daemon=True).start()

    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Victron OLED Simulator v2")

    font_lg = pygame.font.SysFont('Courier', 13, bold=True)
    font_sm = pygame.font.SysFont('Courier', 8)
    font_ui = pygame.font.SysFont('Helvetica', 12)

    oled_surf    = pygame.Surface((OLED_W, OLED_H))
    screen_idx   = 0
    NUM_SCREENS  = 4
    SCREEN_NAMES = ["1 · Battery", "2 · Solar", "3 · State / load", "4 · Yield"]
    clock        = pygame.time.Clock()
    last_cycle   = time.time()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()

        # Auto-cycle
        now     = time.time()
        elapsed = now - last_cycle
        if elapsed >= CYCLE_SECONDS:
            screen_idx = (screen_idx + 1) % NUM_SCREENS
            last_cycle = now
            elapsed    = 0

        with data_lock:
            d = dict(data)

        draw_screen(oled_surf, font_lg, font_sm, screen_idx, d)

        scaled = pygame.transform.scale(oled_surf, (WIN_W, OLED_H * SCALE))
        screen.fill(DKGRAY)
        pygame.draw.rect(screen, (50, 50, 50), (0, 0, WIN_W, OLED_H * SCALE + 4))
        screen.blit(scaled, (0, 2))
        pygame.draw.rect(screen, GRAY, (0, 0, WIN_W, OLED_H * SCALE + 4), 1)

        # Progress bar just below OLED
        draw_progress(screen, elapsed, CYCLE_SECONDS, OLED_H * SCALE + 6)

        # Screen label
        screen.blit(
            font_ui.render(f"Screen: {SCREEN_NAMES[screen_idx]}", True, YELLOW),
            (8, OLED_H * SCALE + 14)
        )

        pygame.display.flip()
        clock.tick(20)

if __name__ == '__main__':
    main()
