#!/usr/bin/env python3
"""Print the breathing rate and heart rate TLVs from the data port.

usage: read_rates.py [data-port] [seconds]
"""

import struct
import sys
import time

import serial

DATA_BAUD = 921600
DEFAULT_PORT = "/dev/ttyACM1"
MAGIC = b"\x02\x01\x04\x03\x06\x05\x08\x07"
HEADER_LEN = 40
TLV_BREATHING = 10
TLV_HEARTRATE = 11


def packets(ser, duration):
    """Yield (frameNumber, {tlvType: payload}) for each well-formed packet."""
    buf = bytearray()
    deadline = time.time() + duration
    while time.time() < deadline:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
        while True:
            start = buf.find(MAGIC)
            if start < 0:
                del buf[:-len(MAGIC)]
                break
            if len(buf) - start < HEADER_LEN:
                del buf[:start]
                break
            hdr = buf[start:start + HEADER_LEN]
            total_len = struct.unpack_from("<I", hdr, 12)[0]
            frame_num = struct.unpack_from("<I", hdr, 20)[0]
            num_tlvs = struct.unpack_from("<I", hdr, 32)[0]
            if total_len < HEADER_LEN or total_len > 65536:
                del buf[:start + len(MAGIC)]
                continue
            if len(buf) - start < total_len:
                del buf[:start]
                break
            packet = bytes(buf[start:start + total_len])
            del buf[:start + total_len]

            tlvs = {}
            off = HEADER_LEN
            for _ in range(num_tlvs):
                if off + 8 > total_len:
                    break
                ttype, tlen = struct.unpack_from("<II", packet, off)
                off += 8
                if off + tlen > total_len:
                    break
                tlvs[ttype] = packet[off:off + tlen]
                off += tlen
            yield frame_num, tlvs


# Sentinels the firmware reports in place of a rate (see vitalsigns.c).
STATES = {
    -1.0: "<warming up>",
    -2.0: "<selecting bin>",
    -3.0: "<filling window>",
    -4.0: "<spectrum NaN/Inf>",
    -5.0: "<spectrum all zero>",
    -6.0: "<no subject>",
     0.0: "<0.0: STALE IMAGE?>",
}


STAGES = {
    1: "atan2 output",
    2: "phase unwrap",
    3: "DC phase removal",
    4: "phase differencing",
    5: "breathing filter",
    6: "heartbeat filter",
    7: "FFT input buffer",
    8: "spectrum",
}


def fmt(v):
    if v != v:
        return "NaN"
    if v in STATES:
        return STATES[v]
    if -108.0 <= v <= -101.0 and float(v).is_integer():
        return f"<NaN@{STAGES[int(-v) - 100]}>"
    return f"{v:7.2f} /min"


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0

    last = None
    with serial.Serial(port, baudrate=DATA_BAUD, timeout=0.1) as ser:
        for frame_num, tlvs in packets(ser, duration):
            br = tlvs.get(TLV_BREATHING)
            hr = tlvs.get(TLV_HEARTRATE)
            if br is None or hr is None or len(br) != 4 or len(hr) != 4:
                continue
            br = struct.unpack("<f", br)[0]
            hr = struct.unpack("<f", hr)[0]
            if (br, hr) == last:
                continue          # only print transitions, not 10 Hz of repeats
            last = (br, hr)
            left = fmt(br)
            right = f"bin {int(hr)}" if left.startswith("<NaN@") else fmt(hr)
            print(f"frame {frame_num:6d}  {left:28s} {right}")


if __name__ == "__main__":
    main()
