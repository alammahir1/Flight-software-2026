#!/usr/bin/env python3
"""
CanSat 2026 — Team 1079 — Ground Station Terminal
--------------------------------------------------
- Reads telemetry from CanSat over COM port and prints it live.
- Lets you type and send commands manually.
- Type  !SIM <file.csv>   to start streaming simulated pressure values.
- Type  !SIM DEMO         to use the built-in demo pressure profile.
- Type  !STOP             to stop SIM streaming.
- Type  !EXIT             to quit.
"""

import serial
import serial.tools.list_ports
import threading
import time
import csv
import sys
import os
from datetime import datetime

# ─────────────────────────── CONFIG ───────────────────────────
TEAM_ID      = "1079"
BAUD_RATE    = 9600          # XBee baud — change to 115200 for USB Serial
READ_TIMEOUT = 0.1           # seconds
SIM_INTERVAL = 1.0           # seconds between SIMP commands
LOG_FILE     = f"ground_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# Built-in demo pressure profile (~600 m apogee and back)
DEMO_PROFILE = [
    101325, 101000, 100500, 100000, 99000, 98000, 97000, 96000,
    95000,  94000,  93000,  92000,  91500, 91000, 90500, 90000,
    89500,  89200,  89000,  89000,  89200, 89500, 90000, 90500,
    91000,  91500,  92000,  93000,  94000, 95000, 96000, 97000,
    98000,  99000, 100000, 100500, 101000, 101325,
]

# ─────────────────────────── GLOBALS ──────────────────────────
ser            = None
sim_running    = False
sim_thread     = None
log_fh         = None
lock           = threading.Lock()


# ─────────────────────────── HELPERS ──────────────────────────
def ts():
    return datetime.now().strftime("%H:%M:%S")

def print_rx(line):
    """Print a received telemetry line, prefixed and logged."""
    print(f"\r\033[32m[RX {ts()}] {line}\033[0m")
    print("> ", end="", flush=True)
    if log_fh:
        log_fh.write(line + "\n")
        log_fh.flush()

def print_info(msg):
    print(f"\r\033[36m[-- {ts()}] {msg}\033[0m")
    print("> ", end="", flush=True)

def print_err(msg):
    print(f"\r\033[31m[!! {ts()}] {msg}\033[0m")
    print("> ", end="", flush=True)

def print_sent(cmd):
    print(f"\r\033[33m[TX {ts()}] {cmd}\033[0m")
    print("> ", end="", flush=True)


# ─────────────────────────── SERIAL ───────────────────────────
def send(cmd_str):
    """Send a command string, appending \\r\\n."""
    if ser is None or not ser.is_open:
        print_err("Not connected.")
        return
    try:
        line = (cmd_str.strip() + "\r\n").encode()
        ser.write(line)
        print_sent(cmd_str.strip())
    except serial.SerialException as e:
        print_err(f"Send failed: {e}")

def reader_thread():
    """Background thread: drain the serial RX buffer and print lines."""
    buf = ""
    while True:
        if ser is None or not ser.is_open:
            time.sleep(0.2)
            continue
        try:
            data = ser.read(256).decode("utf-8", errors="replace")
            if data:
                buf += data
                while "\n" in buf or "\r" in buf:
                    for sep in ("\r\n", "\n", "\r"):
                        idx = buf.find(sep)
                        if idx != -1:
                            line = buf[:idx].strip()
                            buf  = buf[idx + len(sep):]
                            if line:
                                print_rx(line)
                            break
        except serial.SerialException:
            time.sleep(0.2)
        except Exception:
            time.sleep(0.1)


# ─────────────────────────── SIM STREAMING ────────────────────
def sim_stream(pressures):
    """Send CMD,1079,SIMP,<Pa> once per second until stopped."""
    global sim_running
    print_info(f"SIM streaming started — {len(pressures)} pressure values @ 1 Hz")
    print_info("Type !STOP to halt streaming at any time.")

    for pa in pressures:
        if not sim_running:
            break
        send(f"CMD,{TEAM_ID},SIMP,{pa}")
        # Sleep in small chunks so !STOP responds quickly
        for _ in range(int(SIM_INTERVAL / 0.05)):
            if not sim_running:
                break
            time.sleep(0.05)

    sim_running = False
    print_info("SIM streaming complete (or stopped).")

def load_pressures(path):
    """
    Load pressure values (Pa) from a CSV file.
    Accepts:
      - Single-column CSV: one Pa value per row.
      - Two-column CSV:    time, pressure  (header optional).
      - Competition format: any column named 'PRESSURE' or 'pressure' (kPa or Pa).
    Returns a list of ints.
    """
    pressures = []
    with open(path, newline="") as f:
        sample = f.read(1024)
        f.seek(0)
        # Detect if there's a header
        has_header = not sample.lstrip().split("\n")[0].replace(",","").replace(".","").strip().isdigit()
        reader = csv.reader(f)
        header = None
        if has_header:
            header = [h.strip().upper() for h in next(reader)]

        pressure_col = None
        if header:
            for i, h in enumerate(header):
                if "PRESSURE" in h or "PRESS" in h or "PA" == h or "KPA" in h:
                    pressure_col = i
                    break

        for row in reader:
            if not row:
                continue
            try:
                if pressure_col is not None:
                    val = float(row[pressure_col].strip())
                elif len(row) == 1:
                    val = float(row[0].strip())
                else:
                    # Two-column: take last column as pressure
                    val = float(row[-1].strip())
                # Auto-detect kPa vs Pa (values < 2000 are assumed kPa)
                if val < 2000:
                    val *= 1000.0
                pressures.append(int(val))
            except (ValueError, IndexError):
                continue

    return pressures

def start_sim(arg):
    """Parse !SIM argument and start the streaming thread."""
    global sim_running, sim_thread

    if sim_running:
        print_err("SIM already running. Type !STOP first.")
        return

    arg = arg.strip()
    if arg.upper() == "DEMO":
        pressures = DEMO_PROFILE
        print_info("Using built-in demo profile (600 m apogee).")
    elif arg == "":
        print_err("Usage:  !SIM <filename.csv>   or   !SIM DEMO")
        return
    else:
        if not os.path.isfile(arg):
            print_err(f"File not found: {arg}")
            return
        try:
            pressures = load_pressures(arg)
        except Exception as e:
            print_err(f"Could not read file: {e}")
            return
        if not pressures:
            print_err("No valid pressure values found in file.")
            return
        print_info(f"Loaded {len(pressures)} values from {arg}")

    # Auto-send the SIM preamble
    print_info("Sending SIM setup sequence...")
    send(f"CMD,{TEAM_ID},SIM,ENABLE")
    time.sleep(0.3)
    send(f"CMD,{TEAM_ID},SIM,ACTIVATE")
    time.sleep(0.3)
    send(f"CMD,{TEAM_ID},SIMP,{pressures[0]}")
    time.sleep(0.5)
    send(f"CMD,{TEAM_ID},CAL")
    time.sleep(0.3)
    print_info("SIM preamble sent. Starting pressure stream...")

    sim_running = True
    sim_thread = threading.Thread(target=sim_stream, args=(pressures,), daemon=True)
    sim_thread.start()

def stop_sim():
    global sim_running
    if sim_running:
        sim_running = False
        print_info("Stopping SIM stream...")
        time.sleep(0.2)
        send(f"CMD,{TEAM_ID},SIM,DISABLE")
    else:
        print_info("No SIM stream running.")


# ─────────────────────────── PORT SELECTION ───────────────────
def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("  No serial ports found.")
        return []
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  —  {p.description}")
    return ports

def connect(port, baud=BAUD_RATE):
    global ser
    try:
        if ser and ser.is_open:
            ser.close()
        ser = serial.Serial(port, baud, timeout=READ_TIMEOUT)
        print_info(f"Connected to {port} @ {baud} baud")
        return True
    except serial.SerialException as e:
        print_err(f"Could not open {port}: {e}")
        return False


# ─────────────────────────── MAIN ─────────────────────────────
def main():
    global ser, log_fh

    print("=" * 60)
    print("  CanSat 2026 — Team 1079 — Ground Station Terminal")
    print("=" * 60)

    # Open log file
    log_fh = open(LOG_FILE, "w")
    print(f"  Logging telemetry to: {LOG_FILE}")
    print()

    # Port selection
    print("Available serial ports:")
    ports = list_ports()
    if not ports:
        port_str = input("Enter COM port manually (e.g. COM3 or /dev/ttyUSB0): ").strip()
    else:
        choice = input(f"Select port [0-{len(ports)-1}] or type manually: ").strip()
        try:
            port_str = ports[int(choice)].device
        except (ValueError, IndexError):
            port_str = choice

    baud_input = input(f"Baud rate [{BAUD_RATE}]: ").strip()
    baud = int(baud_input) if baud_input.isdigit() else BAUD_RATE

    if not connect(port_str, baud):
        print_err("Could not connect. Exiting.")
        sys.exit(1)

    # Start background reader
    t = threading.Thread(target=reader_thread, daemon=True)
    t.start()

    print()
    print("  Type commands to send (e.g. CMD,1079,CX,ON)")
    print("  Special commands:")
    print("    !SIM <file.csv>  — stream pressure profile from CSV")
    print("    !SIM DEMO        — stream built-in demo profile")
    print("    !STOP            — stop SIM streaming")
    print("    !PORT            — reconnect to a different port")
    print("    !EXIT            — quit")
    print()

    while True:
        try:
            user_input = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            stop_sim()
            break

        if not user_input:
            continue

        upper = user_input.upper()

        if upper == "!EXIT" or upper == "!QUIT":
            stop_sim()
            break

        elif upper.startswith("!SIM"):
            arg = user_input[4:].strip()
            start_sim(arg)

        elif upper == "!STOP":
            stop_sim()

        elif upper == "!PORT":
            stop_sim()
            print("Available serial ports:")
            ports = list_ports()
            if ports:
                choice = input(f"Select [0-{len(ports)-1}] or type manually: ").strip()
                try:
                    new_port = ports[int(choice)].device
                except (ValueError, IndexError):
                    new_port = choice
            else:
                new_port = input("Enter port: ").strip()
            baud_input = input(f"Baud rate [{baud}]: ").strip()
            baud = int(baud_input) if baud_input.isdigit() else baud
            connect(new_port, baud)

        else:
            # Regular command — send as-is
            send(user_input)

    if ser and ser.is_open:
        ser.close()
    if log_fh:
        log_fh.close()
    print(f"Session log saved to: {LOG_FILE}")


if __name__ == "__main__":
    main()
