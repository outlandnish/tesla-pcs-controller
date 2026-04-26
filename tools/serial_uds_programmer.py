#!/usr/bin/env python3
"""
Host uploader for src/uds_programmer.cpp.

Usage (raw payload):
    python3 tools/serial_uds_programmer.py \
        --port /dev/tty.usbmodemXXXX \
        --file firmware.bin \
        --address 0x88000 \
        --memid 0x01

Usage (BHX container):
    python3 tools/serial_uds_programmer.py \
        --port /dev/tty.usbmodemXXXX \
        --bhx firmware.bhx \
        --memid 0x01
"""

import argparse
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("pyserial is required: pip install pyserial", file=sys.stderr)
    sys.exit(1)


REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    from bhx_parser import parse_bhx
except ImportError:
    parse_bhx = None


def read_line(ser: serial.Serial, timeout_s: float = 5.0) -> str:
    end = time.time() + timeout_s
    buf = bytearray()
    while time.time() < end:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\n":
            line = buf.decode("ascii", errors="replace").strip()
            if line:
                return line
            buf.clear()
            continue
        if b != b"\r":
            buf.extend(b)
    raise TimeoutError("Timed out waiting for line from programmer")


def send_cmd(ser: serial.Serial, cmd: str, timeout_s: float = 10.0) -> str:
    ser.write((cmd + "\n").encode("ascii"))
    ser.flush()
    line = read_line(ser, timeout_s)
    return line


def expect_ok(line: str, context: str) -> None:
    if not line.startswith("OK"):
        raise RuntimeError(f"{context} failed: {line}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Upload firmware over serial to UDS programmer")
    parser.add_argument("--port", required=True,
                        help="Serial port (e.g. /dev/tty.usbmodemXXXX)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Serial baud rate")
    src_group = parser.add_mutually_exclusive_group(required=True)
    src_group.add_argument("--file", help="Path to raw firmware payload file")
    src_group.add_argument("--bhx", help="Path to BHX container file")
    parser.add_argument(
        "--address", help="Target address in hex (e.g. 0x88000); optional for --bhx")
    parser.add_argument("--chunk", type=int, default=128,
                        help="Bytes per CHUNK command (max 256)")
    parser.add_argument(
        "--memid", help="Optional memory identifier in hex (e.g. 0x01)")
    parser.add_argument(
        "--alfi", help="Optional ALFI override in hex (e.g. 0x45)")
    args = parser.parse_args()

    if args.chunk < 1 or args.chunk > 256:
        raise ValueError("--chunk must be in range 1..256")

    payload: bytes
    address: int | None = None

    if args.file:
        firmware_path = Path(args.file)
        if not firmware_path.exists() or not firmware_path.is_file():
            raise FileNotFoundError(
                f"Firmware file not found: {firmware_path}")
        payload = firmware_path.read_bytes()
    else:
        if parse_bhx is None:
            raise RuntimeError(
                "BHX support unavailable (failed to import bhx_parser.py)")

        bhx_path = Path(args.bhx)
        if not bhx_path.exists() or not bhx_path.is_file():
            raise FileNotFoundError(f"BHX file not found: {bhx_path}")

        report = parse_bhx(bhx_path)
        header = report["header"]
        if not header.get("payload_size_matches", False):
            raise RuntimeError("BHX payload size mismatch")
        if not header.get("repeated_payload_size_matches", False):
            raise RuntimeError("BHX repeated payload size mismatch")
        if not header.get("payload_crc_matches", False):
            raise RuntimeError("BHX payload CRC mismatch")

        payload = bhx_path.read_bytes()[32:]
        address = int(header["target"])

        print(
            "[*] BHX parsed "
            f"target=0x{address:X} payload={len(payload)} "
            f"global_magic={header['global_magic']} segment_magic={header['segment_magic']}"
        )

    if args.address:
        address = int(args.address, 16)

    if address is None:
        raise ValueError("--address is required when using --file")

    size = len(payload)
    if size == 0:
        raise ValueError("Firmware file is empty")

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        time.sleep(0.3)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Device may emit boot banners before command responses.
        send_cmd(ser, "PING", timeout_s=3.0)

        print("[*] PREP (programming session + unlock)")
        line = send_cmd(ser, "PREP", timeout_s=10.0)
        expect_ok(line, "PREP")

        addr_hex = f"{address:X}"
        size_hex = f"{size:X}"

        start_parts = ["START", addr_hex, size_hex]
        if args.memid:
            start_parts.append(args.memid.lower().replace("0x", ""))
        if args.alfi:
            if not args.memid:
                start_parts.append("0")
            start_parts.append(args.alfi.lower().replace("0x", ""))

        print(f"[*] START addr=0x{addr_hex} size=0x{size_hex}")
        line = send_cmd(ser, " ".join(start_parts), timeout_s=10.0)
        expect_ok(line, "START")

        sent = 0
        total = size
        print(f"[*] Sending {total} bytes in {args.chunk}-byte chunks")
        while sent < total:
            chunk = payload[sent: sent + args.chunk]
            line = send_cmd(
                ser, f"CHUNK {chunk.hex().upper()}", timeout_s=10.0)
            expect_ok(line, "CHUNK")
            sent += len(chunk)
            if sent % (args.chunk * 16) == 0 or sent == total:
                pct = (sent * 100.0) / total
                print(f"    {sent}/{total} ({pct:.1f}%)")

        print("[*] END")
        line = send_cmd(ser, "END", timeout_s=10.0)
        expect_ok(line, "END")

        print("[+] Upload complete")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001
        print(f"[!] {exc}", file=sys.stderr)
        raise SystemExit(1)
