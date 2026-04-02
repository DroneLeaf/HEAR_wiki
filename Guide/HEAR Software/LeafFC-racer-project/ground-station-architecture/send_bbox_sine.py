#!/usr/bin/env python3
"""
Send sinusoidal bbox values over CRSF for latency testing.

x and y follow sine/cosine waves; w and h encode a millisecond
timestamp (mod 2^32) so the receiver can measure one-way latency.
"""
import argparse
import math
import struct
import time
import serial

# ---------------- CRSF helpers ----------------

CRSF_SYNC = 0xEA
CRSF_FRAMETYPE_MAVLINK = 0xAA


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
    payload = struct.pack("<HHHH", x, y, w, h)
    body = bytes([CRSF_FRAMETYPE_MAVLINK]) + payload
    crc = crc8_d5(body)
    length = len(body) + 1
    return bytes([CRSF_SYNC, length]) + body + bytes([crc])


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Send sinusoidal bbox over CRSF for latency testing"
    )
    parser.add_argument("--port", required=True, help="Aircraft-side serial port")
    parser.add_argument("--baud", type=int, default=416666)
    parser.add_argument("--rate", type=float, default=50.0, help="Send rate in Hz")
    parser.add_argument("--freq", type=float, default=1.0, help="Sine wave frequency in Hz")
    parser.add_argument("--cx", type=int, default=320, help="X center")
    parser.add_argument("--cy", type=int, default=180, help="Y center")
    parser.add_argument("--ax", type=int, default=200, help="X amplitude")
    parser.add_argument("--ay", type=int, default=100, help="Y amplitude")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0)
    period = 1.0 / args.rate
    t0 = time.monotonic()

    print(
        f"Sending sinusoidal bbox on {args.port} @ {args.baud}  "
        f"rate={args.rate} Hz  sine_freq={args.freq} Hz"
    )
    try:
        while True:
            now = time.monotonic()
            elapsed = now - t0

            # sinusoidal bbox centre
            x = int(args.cx + args.ax * math.sin(2 * math.pi * args.freq * elapsed))
            y = int(args.cy + args.ay * math.cos(2 * math.pi * args.freq * elapsed))
            x = max(0, min(65535, x))
            y = max(0, min(65535, y))

            # encode send timestamp (ms since t0, mod 2^32) into w(lo) and h(hi)
            ts_ms = int(elapsed * 1000) & 0xFFFFFFFF
            w = ts_ms & 0xFFFF
            h = (ts_ms >> 16) & 0xFFFF

            frame = build_crsf_bbox_frame(x, y, w, h)
            ser.write(frame)

            print(
                f"t={elapsed:7.3f}s  x={x:5d} y={y:5d}  "
                f"ts_ms={ts_ms:10d}  bytes={frame.hex()}"
            )
            time.sleep(period)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
