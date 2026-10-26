#!/usr/bin/env python3
"""Log the vital-signs diagnostic stream from the data port to a CSV.

With VITALSIGNS_DIAG = 1 in vitalsigns.c the firmware reuses the two
vital-signs TLVs to carry, per frame:
    TLV 10 -> differenced phase at the locked range bin (pre-filter)
    TLV 11 -> the same sample after the breathing bandpass (post-filter)

usage: log_phase.py <out.csv> [data-port] [seconds]
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


def main():
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <out.csv> [data-port] [seconds]")
        sys.exit(1)

    out_path = sys.argv[1]
    port = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_PORT
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 90.0

    buf = bytearray()
    rows = []
    total_bytes = 0
    packets = 0
    deadline = time.time() + duration

    with serial.Serial(port, baudrate=DATA_BAUD, timeout=0.1) as ser:
        print(f"logging {duration:.0f} s from {port} -> {out_path}")
        while time.time() < deadline:
            chunk = ser.read(4096)
            if chunk:
                buf += chunk
                total_bytes += len(chunk)

            while True:
                start = buf.find(MAGIC)
                if start < 0:
                    # Keep a tail in case the magic straddles two reads.
                    del buf[:-len(MAGIC)]
                    break
                if len(buf) - start < HEADER_LEN:
                    del buf[:start]
                    break

                hdr = buf[start:start + HEADER_LEN]
                # Header after the 8-byte magic: version, totalPacketLen,
                # platform, frameNumber, timeCpuCycles, numDetectedObj,
                # numTLVs, subFrameNumber.
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
                packets += 1

                off = HEADER_LEN
                phase = None
                bin_idx = None
                for _ in range(num_tlvs):
                    if off + 8 > total_len:
                        break
                    ttype, tlen = struct.unpack_from("<II", packet, off)
                    off += 8
                    if off + tlen > total_len:
                        break
                    if ttype == TLV_BREATHING and tlen == 4:
                        phase = struct.unpack_from("<f", packet, off)[0]
                    elif ttype == TLV_HEARTRATE and tlen == 4:
                        bin_idx = struct.unpack_from("<f", packet, off)[0]
                    off += tlen

                if phase is not None and bin_idx is not None:
                    rows.append((frame_num, phase, bin_idx))

    with open(out_path, "w") as f:
        f.write("frame,pre,post\n")
        for frame_num, phase, bin_idx in rows:
            f.write(f"{frame_num},{phase!r},{bin_idx!r}\n")

    print(f"{total_bytes} bytes read, {packets} packets parsed, "
          f"{len(rows)} frames written to {out_path}")
    if rows:
        pre = [p for _, p, _ in rows]
        post = [q for _, _, q in rows]
        print(f"pre-filter  range [{min(pre):+.4f}, {max(pre):+.4f}]")
        print(f"post-filter range [{min(post):+.4f}, {max(post):+.4f}]")


if __name__ == "__main__":
    main()
