#!/usr/bin/env python3

import sys
import time

import serial

COMMAND_BAUD = 115200          # mss_main.c: config->commandBaudRate
DEFAULT_PORT = "/dev/ttyACM0"
REPLY_TIMEOUT = 15.0           # sensorStart runs RF calibration; allow for it
CHAR_DELAY = 0.002             # inter-character gap; see send_command()


def read_reply(ser, timeout=REPLY_TIMEOUT):
    # Collect output until the prompt returns AND the line goes quiet.
    # The CLI emits its prompt more than once per command, so stopping at the
    # first prompt leaves bytes in flight; those then corrupt the *next* command
    # (the sensor drops its leading characters). Keep draining until nothing has
    # arrived for a short settle window.
    
    deadline = time.time() + timeout
    buf = ""

    # Wait for an actual terminator word, not just the prompt: the previous
    # command's trailing prompt is often still in the buffer, and slow commands
    # (sensorStart runs RF calibration for several seconds) would otherwise be
    # declared failed before their reply arrives.
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1).decode("utf-8", errors="replace")
        if chunk:
            buf += chunk
            if ("Done" in buf) or ("Error" in buf) or ("Ignored" in buf) \
                    or ("not recognized" in buf):
                break
        else:
            time.sleep(0.01)
    return buf


def send_command(ser, cmd):
    # Send one character at a time with a short gap. The demo's CLI has no flow
    # control and drops bytes that arrive while it is re-arming its UART read,
    # which shows up as mangled commands ("flushCfg" arriving as "fg"). Pacing
    # the characters is the only reliable way to avoid it.
    #
    # No input flush here: the port must already be drained by the previous
    # read_reply, and flushing now would race with echo still arriving.
    for ch in (cmd + "\r\n").encode():
        ser.write(bytes([ch]))
        ser.flush()
        time.sleep(CHAR_DELAY)
    reply = read_reply(ser)
    time.sleep(0.05)   # let the receiver settle before the next command

    ok = ("Done" in reply) or ("Ignored" in reply)
    status = "ok " if ok else "FAIL"
    print(f"  [{status}] {cmd}")
    if not ok:
        # Show what the sensor actually said, with the echo/prompt noise trimmed.
        detail = " | ".join(
            line.strip()
            for line in reply.splitlines()
            if line.strip() and "mmwDemo:/>" not in line and line.strip() != cmd
        )
        print(f"         -> {detail}")
    return ok


def main():
    if len(sys.argv) not in (2, 3):
        print(f"usage: {sys.argv[0]} <cfg-file> [command-port]")
        sys.exit(1)

    cfg_path = sys.argv[1]
    port = sys.argv[2] if len(sys.argv) == 3 else DEFAULT_PORT

    with open(cfg_path) as f:
        commands = [
            line.strip()
            for line in f
            if line.strip() and not line.startswith("%")
        ]

    print(f"Sending {len(commands)} commands from {cfg_path} to {port}")

    failed = []
    with serial.Serial(port, baudrate=COMMAND_BAUD, timeout=0.1) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        for cmd in commands:
            if not send_command(ser, cmd):
                failed.append(cmd)

    if failed:
        print(f"\n{len(failed)} command(s) failed:")
        for cmd in failed:
            print(f"  {cmd}")
        sys.exit(1)

    print("\nAll commands accepted -- sensor should be running.")


if __name__ == "__main__":
    main()
