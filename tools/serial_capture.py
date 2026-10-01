#!/usr/bin/env python3
"""
Record the logger's serial output to a file, with host timestamps.

Typical uses:
  # offload the FRAM log
  python tools/serial_capture.py --port COM5 --send dump --until-end -o log.csv

  # pressure-chamber / thermal test: stream pressure for an hour, typing reference readings
  # (e.g. "50.2 psi" or "fridge in") as you go; each becomes a MARK line in the file
  python tools/serial_capture.py --port COM5 --send "stream 3600" --until-end -o p_test.csv

Every output line is:  <host_time_s>,<line from the device>
Typed notes are saved as:  <host_time_s>,MARK,<your text>
Requires: pip install pyserial
"""

import argparse
import sys
import threading
import time

import serial


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--send", help="console command to send after connecting (e.g. dump, 'stream 600')")
    ap.add_argument("--until-end", action="store_true", help="stop when the device prints END")
    ap.add_argument("-o", "--output", required=True)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(2.0)  # opening the port resets the Pro Mini through DTR; wait for boot
    ser.reset_input_buffer()

    stop = threading.Event()
    lock = threading.Lock()
    out = open(args.output, "w", buffering=1)
    t0 = time.time()

    def write(line):
        with lock:
            out.write(f"{time.time() - t0:.3f},{line}\n")

    def reader():
        while not stop.is_set():
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").rstrip()
            write(line)
            print(line)
            if args.until_end and line == "END":
                stop.set()

    def notes():
        for note in sys.stdin:  # daemon thread: never blocks shutdown
            write("MARK," + note.strip())

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    threading.Thread(target=notes, daemon=True).start()

    if args.send:
        ser.write((args.send + "\n").encode())

    print("capturing; type a note + Enter to add a MARK, Ctrl+C to stop", file=sys.stderr)
    try:
        while not stop.wait(0.2):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        th.join(timeout=1.0)
        ser.close()
        out.close()
        print(f"saved {args.output}", file=sys.stderr)


if __name__ == "__main__":
    main()
