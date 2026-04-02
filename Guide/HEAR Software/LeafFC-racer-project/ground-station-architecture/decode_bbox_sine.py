#!/usr/bin/env python3
"""
Decode sinusoidal bbox values from CRSF and measure one-way latency.

Expects frames produced by send_bbox_sine.py where:
  x, y  = sinusoidal position
  w, h  = sender timestamp in ms (w=lo16, h=hi16)

Both scripts must share a common time reference (same machine,
or clocks synchronised via NTP/PTP).  If they are on the same
machine the reported latency is the true one-way serial latency.
"""
import argparse
import struct
import time
import serial

CRSF_SYNC = 0xEA
CRSF_FRAMETYPE_MAVLINK = 0xAA
BBOX_PAYLOAD_LEN = 8  # 4 × uint16


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
        body = frame[2:-1]
        crc = frame[-1]
        if crc8_d5(body) == crc:
            yield frame
            i = end
        else:
            i += 1

    del buf[:i]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Decode sinusoidal bbox from CRSF and measure latency"
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="Laptop-side telem mirror port")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    crsf_buf = bytearray()

    frame_count = 0
    window_start = time.monotonic()
    hz = 0.0
    t0 = time.monotonic()  # must match the sender's t0 if on the same machine

    latency_sum = 0.0
    latency_min = float("inf")
    latency_max = 0.0
    latency_n = 0

    print(f"Listening on {args.port}  (press Ctrl-C for summary)")
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

                rx_time = time.monotonic()
                x, y, w, h = struct.unpack("<HHHH", payload)

                # reconstruct sender timestamp
                tx_ms = (h << 16) | w
                rx_ms = int((rx_time - t0) * 1000) & 0xFFFFFFFF

                # handle uint32 wraparound
                latency_ms = (rx_ms - tx_ms) & 0xFFFFFFFF
                if latency_ms > 0x7FFFFFFF:
                    latency_ms -= 0x100000000

                latency_sum += latency_ms
                latency_n += 1
                latency_min = min(latency_min, latency_ms)
                latency_max = max(latency_max, latency_ms)

                frame_count += 1
                now = rx_time
                elapsed = now - window_start
                if elapsed >= 1.0:
                    hz = frame_count / elapsed
                    frame_count = 0
                    window_start = now

                print(
                    f"BBOX x={x:5d} y={y:5d}  "
                    f"latency={latency_ms:6.0f} ms  "
                    f"avg={latency_sum / latency_n:6.1f} ms  "
                    f"min={latency_min:.0f} max={latency_max:.0f}  "
                    f"({hz:.1f} Hz)"
                )

    except KeyboardInterrupt:
        print("\n--- Latency summary ---")
        if latency_n:
            print(f"  Samples : {latency_n}")
            print(f"  Min     : {latency_min:.1f} ms")
            print(f"  Max     : {latency_max:.1f} ms")
            print(f"  Avg     : {latency_sum / latency_n:.1f} ms")
        else:
            print("  No frames received.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
