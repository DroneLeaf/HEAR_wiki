#!/usr/bin/env python3
import argparse
import struct
import time
import serial

# ---------------- CRSF helpers ----------------

CRSF_SYNC = 0xEA
CRSF_FRAMETYPE_MAVLINK = 0xAA  # reuse the MAVLink slot in CRSF


def crc8_d5(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ 0xD5
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_crsf_bbox_frame(x: int, y: int, w: int, h: int) -> bytes:
    """
    CRSF frame carrying bbox on the MAVLink type slot:
      [sync][length][type][x_lo][x_hi][y_lo][y_hi][w_lo][w_hi][h_lo][h_hi][crc]

    Total: 12 bytes on the wire.
    """
    payload = struct.pack("<HHHH", x, y, w, h)
    body = bytes([CRSF_FRAMETYPE_MAVLINK]) + payload
    crc = crc8_d5(body)
    length = len(body) + 1  # type + payload + crc
    return bytes([CRSF_SYNC, length]) + body + bytes([crc])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Aircraft-side serial port")
    parser.add_argument("--baud", type=int, default=416666)
    parser.add_argument("--rate", type=float, default=10.0, help="Hz")
    parser.add_argument("--x", type=int, default=320)
    parser.add_argument("--y", type=int, default=180)
    parser.add_argument("--w", type=int, default=64)
    parser.add_argument("--h", type=int, default=48)
    args = parser.parse_args()

    for name, value in [("x", args.x), ("y", args.y), ("w", args.w), ("h", args.h)]:
        if not (0 <= value <= 65535):
            raise ValueError(f"{name} must be in 0..65535")

    ser = serial.Serial(args.port, args.baud, timeout=0)
    period = 1.0 / args.rate

    print(f"Sending bbox over CRSF on {args.port} @ {args.baud}  (12 bytes/frame)")
    try:
        while True:
            crsf = build_crsf_bbox_frame(args.x, args.y, args.w, args.h)
            ser.write(crsf)
            print(f"x={args.x} y={args.y} w={args.w} h={args.h}  bytes={crsf.hex()}")
            time.sleep(period)
    finally:
        ser.close()


if __name__ == "__main__":
    main()