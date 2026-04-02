#!/usr/bin/env python3
import argparse
import struct
import time
import serial

CRSF_SYNC = 0xEA
CRSF_FRAMETYPE_MAVLINK = 0xAA  # reuse the MAVLink slot in CRSF

BBOX_PAYLOAD_LEN = 8  # 4 × uint16 (x, y, w, h)


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


def parse_crsf_frames(buf: bytearray):
    """
    Yield CRSF frames from a rolling buffer.
    """
    i = 0
    while i <= len(buf) - 4:
        if buf[i] != CRSF_SYNC:
            i += 1
            continue

        length = buf[i + 1]
        if not (2 <= length <= 62):
            i += 1
            continue

        end = i + 2 + length
        if end > len(buf):
            break

        frame = bytes(buf[i:end])
        body = frame[2:-1]   # type + payload
        crc = frame[-1]
        if crc8_d5(body) == crc:
            yield frame
            i = end
        else:
            i += 1

    del buf[:i]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0", help="Laptop-side telem mirror port")
    parser.add_argument("--baud", type=int, default=115200, help="USB ACM speed is usually ignored")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    crsf_buf = bytearray()

    frame_count = 0
    window_start = time.monotonic()
    hz = 0.0

    print(f"Listening on {args.port}")
    try:
        while True:
            data = ser.read(4096)
            if not data:
                continue

            crsf_buf.extend(data)

            for frame in parse_crsf_frames(crsf_buf):
                body = frame[2:-1]
                ftype = body[0]
                payload = body[1:]

                if ftype != CRSF_FRAMETYPE_MAVLINK:
                    continue

                if len(payload) != BBOX_PAYLOAD_LEN:
                    continue

                x, y, w, h = struct.unpack("<HHHH", payload)
                frame_count += 1

                now = time.monotonic()
                elapsed = now - window_start
                if elapsed >= 1.0:
                    hz = frame_count / elapsed
                    frame_count = 0
                    window_start = now

                print(
                    f"BBOX x={x} y={y} w={w} h={h}  "
                    f"({hz:.1f} Hz)"
                )
    finally:
        ser.close()


if __name__ == "__main__":
    main()