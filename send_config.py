#!/usr/bin/env python3
"""Send an mmWave demo .cfg to the sensor's command port, one line at a time.

The CLI echoes and parses asynchronously, so each command must be sent on its
own and acknowledged ("Done") before the next one goes out -- batching lines
causes them to collide and be rejected.

usage: send_config.py <cfg-file> [command-port]
"""

import sys
import time

import serial

COMMAND_BAUD = 115200          # mss_main.c: config->commandBaudRate
DEFAULT_PORT = "/dev/ttyACM0"
REPLY_TIMEOUT = 5.0            # sensorStart takes noticeably longer than the rest


def read_reply(ser, timeout=REPLY_TIMEOUT):
    """Collect output until the prompt returns AND the line goes quiet.

    The CLI emits its prompt more than once per command, so stopping at the
    first prompt leaves bytes in flight; those then corrupt the *next* command
    (the sensor drops its leading characters). Keep draining until nothing has
    arrived for a short settle window.
    """
    deadline = time.time() + timeout
    buf = ""
    seen_prompt = False
    quiet_since = None

    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1).decode("utf-8", errors="replace")
        if chunk:
            buf += chunk
            quiet_since = None
            if "mmwDemo:/>" in buf:
                seen_prompt = True
        else:
            now = time.time()
            if quiet_since is None:
                quiet_since = now
            elif seen_prompt and (now - quiet_since) > 0.15:
                break
            time.sleep(0.01)
    return buf


def send_command(ser, cmd):
    # No input flush here: the port must already be drained by the previous
    # read_reply, and flushing now would race with echo still arriving.
    ser.write((cmd + "\r\n").encode())
    ser.flush()
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
