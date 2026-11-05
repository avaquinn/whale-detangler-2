#!/usr/bin/env python3
"""Decode the dedicated-card raw SD overflow region into CSV.

First make a full card image with a disk-imaging tool, then run:
  python tools/decode_raw_sd.py card.img -o overflow.csv

The firmware starts raw records at sector 2048. Each subsequent record occupies
one 512-byte sector. This tool emits acceleration R rows and full SMP snapshot
rows; other record types are retained as compact RAW rows.
"""
import argparse
import struct

SECTOR_SIZE = 512
START_SECTOR = 2048
MARKER = 0xA5
REC_BLOCK = 4
SAMPLE = 1


def crc16_xmodem(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image", help="full raw image of the dedicated SD card")
    parser.add_argument("-o", "--output", required=True)
    parser.add_argument("--start-sector", type=int, default=START_SECTOR)
    args = parser.parse_args()

    records = 0
    with open(args.image, "rb") as image, open(args.output, "w", newline="") as out:
        out.write("# R,boot,t_ms,ax_mg,ay_mg,az_mg\n")
        out.write("# SMP,boot,seq,t_ms,state,phase,ax_mg,ay_mg,az_mg,motion_mg,temp_raw,p_raw,depth_cm,mv,soc_x100,flags\n")
        image.seek(args.start_sector * SECTOR_SIZE)
        sector = args.start_sector
        while block := image.read(SECTOR_SIZE):
            if len(block) != SECTOR_SIZE or block[0] != MARKER:
                break
            marker, kind, length, boot, sequence = struct.unpack_from("<BBBHI", block)
            total = 9 + length + 2
            if length > 128 or total > SECTOR_SIZE:
                break
            frame = block[:9 + length]
            stored_crc = struct.unpack_from("<H", block, 9 + length)[0]
            if crc16_xmodem(frame[1:]) != stored_crc:
                break
            payload = frame[9:]
            if kind == REC_BLOCK and len(payload) >= 5:
                t0, count = struct.unpack_from("<IB", payload)
                count = min(count, (len(payload) - 5) // 8)
                for sample in range(count):
                    dt, ax, ay, az = struct.unpack_from("<Hhhh", payload, 5 + sample * 8)
                    out.write(f"R,{boot},{t0 + dt},{ax * 4},{ay * 4},{az * 4}\n")
            elif kind == SAMPLE and len(payload) == 28:
                (t_ms, ax, ay, az, motion_mg, temp_raw, pressure_raw, depth_cm,
                 voltage_mv, soc_x100, flags, state, phase) = struct.unpack("<IhhhHhihHHHBB", payload)
                out.write(f"SMP,{boot},{sequence},{t_ms},{state},{phase},{ax * 4},{ay * 4},{az * 4},"
                          f"{motion_mg},{temp_raw},{pressure_raw},{depth_cm},{voltage_mv},{soc_x100},{flags}\n")
            else:
                out.write(f"RAW,{kind},{boot},{sequence},{payload.hex()}\n")
            records += 1
            sector += 1

    print(f"decoded {records} raw sectors starting at sector {args.start_sector}")


if __name__ == "__main__":
    main()
