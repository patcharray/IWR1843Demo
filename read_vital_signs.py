#!/usr/bin/env python3

import math
import struct
import sys
import serial

MAGIC_WORD = struct.pack("<4H", 0x0102, 0x0304, 0x0506, 0x0708)
HEADER_STRUCT = "<4H8I"
HEADER_LEN = struct.calcsize(HEADER_STRUCT)

TLV_HEADER_STRUCT = "<2I"
TLV_HEADER_LEN = struct.calcsize(TLV_HEADER_STRUCT)

TLV_DETECTED_POINTS = 1
TLV_RANGE_PROFILE = 2
TLV_NOISE_PROFILE = 3
TLV_AZIMUT_STATIC_HEAT_MAP = 4
TLV_RANGE_DOPPLER_HEAT_MAP = 5
TLV_STATS = 6
TLV_DETECTED_POINTS_SIDE_INFO = 7
TLV_AZIMUT_ELEVATION_STATIC_HEAT_MAP = 8
TLV_TEMPERATURE_STATS = 9
TLV_BREATHING_RATE = 10
TLV_HEARTRATE = 11

DATA_PORT_BAUD = 921600

RANGE_RESOLUTION = 0.168

def sync_to_magic_word(ser):
    window = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            continue
        window += b
        if len(window) > len(MAGIC_WORD):
            del window[0]
        if bytes(window) == MAGIC_WORD:
            return


def read_frame(ser):
    sync_to_magic_word(ser)

    rest = ser.read(HEADER_LEN - len(MAGIC_WORD))
    header = MAGIC_WORD + rest
    (_,_,_,_, _version, _totalPacketLen, _platform, _frameNumber, _timeCpuCycles, _numDetectedObj, numTLVs, _subFrameNumber) = struct.unpack(HEADER_STRUCT, header)

    for _ in range(numTLVs):
        tlv_header = ser.read(TLV_HEADER_LEN)
        if len(tlv_header) < TLV_HEADER_LEN:
            print("short TLV header, dropping the rest of the frame")
            return

        tlv_type, tlv_length = struct.unpack(TLV_HEADER_STRUCT, tlv_header)
        payload = ser.read(tlv_length)

        if tlv_type == TLV_BREATHING_RATE:
            (breathing_rate, ) = struct.unpack("<f", payload)
            print(f"Breathing rate = {breathing_rate} breathes per minute")

        if tlv_type == TLV_HEARTRATE:
            (heartrate, )  = struct.unpack("<f", payload)
            print(f"Heartrate = {heartrate} beats per minute")


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <data-port, e.g. /dev/ttyACM1 or COM5>")
        sys.exit(1)

    port = sys.argv[1]
    with serial.Serial(port, baudrate=DATA_PORT_BAUD, timeout=1) as ser:
        print(f"Listening on {port} at {DATA_PORT_BAUD} baudrate")
        while True:
            read_frame(ser)



if __name__ == "__main__":
    main()
