#!/usr/bin/env python3
"""Print the two raw floats from the vital-signs TLVs with full precision.

Used while tracing a NaN: the firmware temporarily repurposes the two slots to
carry diagnostic magnitudes, so no interpretation should be applied here.
"""
import sys
from read_rates import packets, TLV_BREATHING, TLV_HEARTRATE
import struct
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM1"
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
last = None
with serial.Serial(port, baudrate=921600, timeout=0.1) as ser:
    for frame, tlvs in packets(ser, dur):
        a, b = tlvs.get(TLV_BREATHING), tlvs.get(TLV_HEARTRATE)
        if not a or not b or len(a) != 4 or len(b) != 4:
            continue
        a = struct.unpack("<f", a)[0]
        b = struct.unpack("<f", b)[0]
        key = (repr(a), repr(b))
        if key == last:
            continue
        last = key
        print(f"frame {frame:6d}  float1={a!r:24s} float2={b!r}")
