#!/usr/bin/env python3
"""
Simple host-side test helper for the Uno fake WSS device.

Usage:
    python test/test_wss_serial.py --port /dev/ttyACM0

The script sends a CLEAR command (which expects a reply) and validates the
response contents, then repeatedly emits streaming frames so the onboard LED
should toggle every 500 ms.
"""

import argparse
import sys
import time

try:
    import serial  # type: ignore
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pyserial is required: pip install pyserial") from exc

END = 0xC0
ESC = 0xDB
END_SUB = 0xDC
ESC_SUB = 0xDD

HOST_ID = 0x00
DEVICE_ID = 0x81

WSS_CLEAR = 0x40
WSS_STREAM_ALL = 0x30


def compute_checksum(data: bytes) -> int:
    total = sum(data)
    total = ((total & 0x00FF) + (total >> 8)) ^ 0xFF
    return total & 0xFF


def build_frame(payload: bytes, sender: int = HOST_ID, target: int = DEVICE_ID) -> bytes:
    raw = bytearray()
    raw.append(sender)
    raw.append(target)
    raw.extend(payload)
    raw.append(0x00)  # checksum placeholder
    raw.append(END)
    raw[-2] = compute_checksum(raw[:-2])

    escaped = bytearray()
    for byte in raw[:-1]:
        if byte == END:
            escaped.extend((ESC, END_SUB))
        elif byte == ESC:
            escaped.extend((ESC, ESC_SUB))
        else:
            escaped.append(byte)

    escaped.append(END)
    return bytes(escaped)


def unescape_frame(preescaped: bytes) -> bytes | None:
    out = bytearray()
    i = 0
    while i < len(preescaped):
        byte = preescaped[i]
        if byte == ESC:
            i += 1
            if i >= len(preescaped):
                return None
            next_byte = preescaped[i]
            if next_byte == END_SUB:
                out.append(END)
            elif next_byte == ESC_SUB:
                out.append(ESC)
            else:
                out.append(next_byte)
        else:
            out.append(byte)
        i += 1
    return bytes(out)


def read_frame(port: serial.Serial, timeout: float = 2.0) -> bytes | None:
    deadline = time.time() + timeout
    buffer = bytearray()
    while time.time() < deadline:
        incoming = port.read(1)
        if not incoming:
            continue
        byte = incoming[0]
        if byte == END:
            if not buffer:
                continue
            frame = unescape_frame(buffer)
            buffer.clear()
            if not frame or len(frame) < 3:
                continue
            expected = frame[-1]
            computed = compute_checksum(frame[:-1])
            if computed == expected:
                return frame
        else:
            buffer.append(byte)
    return None


def format_bytes(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def run_clear_test(port: serial.Serial, iterations: int = 1) -> bool:
    payload = bytes([WSS_CLEAR, 0x01, 0x00])
    overall_ok = True

    for attempt in range(1, iterations + 1):
        if iterations > 1:
            print(f"\nCLEAR attempt {attempt}/{iterations}")

        frame = build_frame(payload)
        print(f"TX CLEAR: {format_bytes(payload)}")
        port.reset_input_buffer()
        start = time.perf_counter()
        port.write(frame)
        port.flush()

        reply = read_frame(port, timeout=2.0)
        if not reply:
            print("RX CLEAR: timeout waiting for reply")
            overall_ok = False
            continue

        latency_ms = (time.perf_counter() - start) * 1000.0
        print(f"RX CLEAR frame: {format_bytes(reply)} (latency {latency_ms:.1f} ms)")

        sender, target = reply[0], reply[1]
        rx_payload = reply[2:-1]
        print(f"RX CLEAR sender=0x{sender:02X} target=0x{target:02X} payload={format_bytes(rx_payload)}")

        ok = (sender == DEVICE_ID and target == HOST_ID and rx_payload == payload)
        print("CLEAR result:", "PASS" if ok else "FAIL")
        overall_ok = overall_ok and ok

    return overall_ok


def run_stream_loop(port: serial.Serial, iterations: int = 10, interval: float = 0.5) -> None:
    on_payload = bytes([WSS_STREAM_ALL, 1, 1, 1, 1, 1, 1])
    off_payload = bytes([WSS_STREAM_ALL, 0, 0, 0, 0, 0, 0])

    for idx in range(iterations):
        payload = on_payload if idx % 2 == 0 else off_payload
        frame = build_frame(payload)
        port.write(frame)
        port.flush()
        state = "ON " if idx % 2 == 0 else "OFF"
        print(f"TX STREAM {state}: {format_bytes(payload)}")
        time.sleep(interval)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Host tester for the Uno fake WSS device.")
    parser.add_argument("--port", required=True, help="Serial port connected to the Uno (e.g. /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument("--clear-only", action="store_true", help="Run the CLEAR round-trip test only")
    parser.add_argument("--clear-iterations", type=int, default=1, help="How many CLEAR round-trips to run")
    parser.add_argument("--stream-iterations", type=int, default=10, help="How many LED toggle messages to send")
    args = parser.parse_args(argv)

    with serial.Serial(args.port, baudrate=args.baud, timeout=0.1) as port:
        time.sleep(2.0)  # allow board reset after opening the port
        port.reset_input_buffer()

        clear_ok = run_clear_test(port, iterations=args.clear_iterations)

        if not args.clear_only:
            run_stream_loop(port, iterations=args.stream_iterations)

    return 0 if clear_ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
