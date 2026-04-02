#!/usr/bin/env python3
import serial
import time
import math
import sys

PORT = "/dev/ttyUSB1"

# SBUS basics
SBUS_BAUD = 100000
SBUS_HEADER = 0x0F
SBUS_FOOTER = 0x00

# Standard RC-ish ranges in SBUS domain
# 172   ~= low
# 992   ~= center
# 1811  ~= high
SBUS_LOW = 172
SBUS_MID = 992
SBUS_HIGH = 1811

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def pack_sbus(ch, failsafe=False, frame_lost=False):
    """
    Pack 16 SBUS channels into one 25-byte frame.
    ch: list of 16 ints, each 0..2047 (11-bit)
    """
    if len(ch) != 16:
        raise ValueError("Need exactly 16 channels")

    ch = [clamp(int(x), 0, 2047) for x in ch]

    data = [0] * 25
    data[0] = SBUS_HEADER

    bit_index = 0
    for c in ch:
        for i in range(11):
            if c & (1 << i):
                byte_index = 1 + (bit_index // 8)
                bit_in_byte = bit_index % 8
                data[byte_index] |= (1 << bit_in_byte)
            bit_index += 1

    flags = 0
    if frame_lost:
        flags |= (1 << 2)
    if failsafe:
        flags |= (1 << 3)

    data[23] = flags
    data[24] = SBUS_FOOTER
    return bytes(data)

def main():
    port = PORT
    if len(sys.argv) > 1:
        port = sys.argv[1]

    ser = serial.Serial(
        port=port,
        baudrate=SBUS_BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_TWO,
        timeout=0
    )

    print(f"Sending SBUS on {port}")
    print("CH1-6 fixed at center, CH7 sweeps, CH8 toggles every second")

    t0 = time.time()
    last_toggle = 0
    ch8_high = False

    try:
        while True:
            t = time.time() - t0

            # Start with all channels centered
            ch = [SBUS_MID] * 16

            # Keep CH1-6 centered so they do not matter
            # Hijack CH7 and CH8 only
            # NOTE: SBUS channel numbering is 1-based to humans, 0-based in list
            # CH7 => index 6
            # CH8 => index 7

            # CH7 sweeps smoothly low -> high -> low
            s = 0.5 * (1.0 + math.sin(2.0 * math.pi * 0.2 * t))
            ch[6] = int(SBUS_LOW + s * (SBUS_HIGH - SBUS_LOW))

            # CH8 toggles every second
            if int(t) != last_toggle:
                last_toggle = int(t)
                ch8_high = not ch8_high
            ch[7] = SBUS_HIGH if ch8_high else SBUS_LOW

            frame = pack_sbus(ch)
            ser.write(frame)

            # Typical SBUS period ~9ms
            time.sleep(0.009)

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

if __name__ == "__main__":
    main()
