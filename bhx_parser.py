#!/usr/bin/env python3

from __future__ import annotations

import argparse
import binascii
import json
import struct
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


@dataclass
class BhxHeader:
    global_magic: str
    global_version: int
    payload_size_in_header: int
    segment_magic: str
    segment_version: int
    target: int
    repeated_payload_size: int
    payload_crc32: int
    file_size: int
    payload_size: int
    payload_size_matches: bool
    repeated_payload_size_matches: bool
    payload_crc_matches: bool


def read_u32be(blob: bytes, offset: int) -> int:
    return struct.unpack_from(">I", blob, offset)[0]


def swap_u16_halves(value: int) -> int:
    return ((value & 0xFFFF) << 16) | ((value >> 16) & 0xFFFF)


def decode_bhx_header(blob: bytes) -> BhxHeader:
    if len(blob) < 32:
        raise ValueError("File is too small to contain a BHX header")

    (
        global_magic,
        global_version,
        payload_size_in_header,
        segment_magic,
        segment_version,
        target,
        repeated_payload_size,
        payload_crc32,
    ) = struct.unpack(">4sII4sIIII", blob[:32])
    payload = blob[32:]
    computed_payload_crc = binascii.crc32(payload) & 0xFFFFFFFF

    return BhxHeader(
        global_magic=global_magic.decode("ascii", errors="replace"),
        global_version=global_version,
        payload_size_in_header=payload_size_in_header,
        segment_magic=segment_magic.decode("ascii", errors="replace"),
        segment_version=segment_version,
        target=target,
        repeated_payload_size=repeated_payload_size,
        payload_crc32=payload_crc32,
        file_size=len(blob),
        payload_size=len(payload),
        payload_size_matches=payload_size_in_header == len(payload),
        repeated_payload_size_matches=repeated_payload_size == len(payload),
        payload_crc_matches=payload_crc32 == computed_payload_crc,
    )


def payload_words(blob: bytes, count: int) -> list[int]:
    payload = blob[32:]
    usable = min(len(payload) // 4, count)
    return [read_u32be(payload, index * 4) for index in range(usable)]


def derive_filename_hints(path: Path) -> dict[str, Any]:
    name = path.stem
    hints: dict[str, Any] = {}
    for part in name.split("_"):
        if (
            len(part) >= 2
            and part[0] in {"P", "A", "U"}
            and part[1:].isdigit()
        ):
            hints[part[0]] = int(part[1:])
        elif part.startswith("CPU") and part[3:].isdigit():
            hints["CPU"] = int(part[3:])
    return hints


def f28377d_flash_sectors() -> list[dict[str, int | str]]:
    return [
        {"name": "Sector 0", "start": 0x00080000, "end": 0x00081FFF},
        {"name": "Sector 1", "start": 0x00082000, "end": 0x00083FFF},
        {"name": "Sector 2", "start": 0x00084000, "end": 0x00085FFF},
        {"name": "Sector 3", "start": 0x00086000, "end": 0x00087FFF},
        {"name": "Sector 4", "start": 0x00088000, "end": 0x0008FFFF},
        {"name": "Sector 5", "start": 0x00090000, "end": 0x00097FFF},
        {"name": "Sector 6", "start": 0x00098000, "end": 0x0009FFFF},
        {"name": "Sector 7", "start": 0x000A0000, "end": 0x000A7FFF},
        {"name": "Sector 8", "start": 0x000A8000, "end": 0x000AFFFF},
        {"name": "Sector 9", "start": 0x000B0000, "end": 0x000B7FFF},
        {"name": "Sector 10", "start": 0x000B8000, "end": 0x000B9FFF},
        {"name": "Sector 11", "start": 0x000BA000, "end": 0x000BBFFF},
        {"name": "Sector 12", "start": 0x000BC000, "end": 0x000BDFFF},
        {"name": "Sector 13", "start": 0x000BE000, "end": 0x000BFFFF},
    ]


def decode_f28377d_span(
    target: int, payload_size: int
) -> dict[str, Any] | None:
    if not 0x00080000 <= target <= 0x000BFFFF:
        return None

    payload_words_16 = (payload_size + 1) // 2
    image_end = target + payload_words_16 - 1
    touched = [
        sector["name"]
        for sector in f28377d_flash_sectors()
        if not (image_end < sector["start"] or target > sector["end"])
    ]
    return {
        "address_unit": "16-bit words",
        "start": f"0x{target:08x}",
        "end": f"0x{image_end:08x}",
        "payload_words_16": payload_words_16,
        "sectors": touched,
    }


def collect_flash_address_like_words(
    payload: bytes,
) -> list[tuple[int, int, int]]:
    matches: list[tuple[int, int, int]] = []
    for offset in range(0, len(payload) - 3, 4):
        raw = read_u32be(payload, offset)
        swapped = swap_u16_halves(raw)
        if 0x00080000 <= swapped <= 0x000BFFFF:
            matches.append((offset, raw, swapped))
    return matches


def trailing_flash_address_run(
    values: list[tuple[int, int, int]],
) -> tuple[int, int | None]:
    if not values:
        return 0, None

    run_count = 1
    stride_words: int | None = None
    for index in range(len(values) - 1, 0, -1):
        offset, _, address = values[index]
        prev_offset, _, prev_address = values[index - 1]
        if offset - prev_offset != 4:
            break
        step = prev_address - address
        if step <= 0:
            break
        if stride_words is None:
            stride_words = step
        elif step != stride_words:
            break
        run_count += 1
    return run_count, stride_words


def decode_c28x_image_analysis(
    payload: bytes, target: int
) -> dict[str, Any] | None:
    span = decode_f28377d_span(target, len(payload))
    if span is None:
        return None

    flash_words = collect_flash_address_like_words(payload)
    start_window_count = sum(
        1 for offset, _, _ in flash_words if offset < 0x40
    )
    run_count, stride_words = trailing_flash_address_run(flash_words)
    includes_flash_begin = target <= 0x00080000
    first_code_offset = 0x20 if len(payload) >= 0x40 else 0
    first_code_address = target + (first_code_offset // 2)
    first_code_preview = payload[first_code_offset:first_code_offset + 16]

    if includes_flash_begin and start_window_count >= 8:
        vector_reason = (
            "image includes flash begin and starts with address-like words"
        )
        likely_vector_table = True
    elif not includes_flash_begin:
        vector_reason = "image starts above flash begin region"
        likely_vector_table = False
    else:
        vector_reason = (
            "image start is code-like rather than a dense pointer block"
        )
        likely_vector_table = False

    return {
        "includes_flash_begin_region": includes_flash_begin,
        "likely_flash_vector_table_present": likely_vector_table,
        "vector_table_reason": vector_reason,
        "flash_address_like_word_count": len(flash_words),
        "flash_address_like_words_in_first_0x40_bytes": start_window_count,
        "trailing_flash_address_run_count": run_count,
        "trailing_flash_address_stride_words": stride_words,
        "first_executable_offset_bytes_candidate": first_code_offset,
        "first_executable_address_candidate": f"0x{first_code_address:08x}",
        "first_executable_preview_hex": first_code_preview.hex(),
    }


def decode_pcs_payload(payload: bytes, path: Path) -> dict[str, Any]:
    words = payload_words(payload, 16)
    if len(words) < 4:
        return {}

    first = words[0]
    second = words[1]
    range_word_6 = words[6] if len(words) > 6 else None
    range_word_7 = words[7] if len(words) > 7 else None
    filename_hints = derive_filename_hints(path)

    result = {
        "component_id_candidate": (first >> 16) & 0xFFFF,
        "a_value_candidate": (first >> 8) & 0xFF,
        "p_value_candidate": first & 0xFF,
        "u_value_candidate": (second >> 16) & 0xFFFF,
        "reserved_words": words[2:4],
        "filename_hints": filename_hints,
        "first_payload_words": [f"0x{word:08x}" for word in words],
    }

    if range_word_6 is not None:
        result["word6_c28x_address_candidate"] = (
            f"0x{swap_u16_halves(range_word_6):08x}"
        )
    if range_word_7 is not None:
        result["word7_c28x_address_candidate"] = (
            f"0x{swap_u16_halves(range_word_7):08x}"
        )

    if range_word_6 is not None and range_word_7 is not None:
        word6_addr = swap_u16_halves(range_word_6)
        word7_addr = swap_u16_halves(range_word_7)
        result["section_words_between_word6_and_word7"] = (
            word7_addr - word6_addr
        )
        result["section_bytes_between_word6_and_word7"] = (
            (word7_addr - word6_addr) * 2
        )

    if len(payload) >= 8:
        tail_penultimate = read_u32be(payload, len(payload) - 8)
        tail_last = read_u32be(payload, len(payload) - 4)
        result["tail_penultimate_word"] = f"0x{tail_penultimate:08x}"
        result["tail_last_word"] = f"0x{tail_last:08x}"

    return result


def decode_payload_hints(
    blob: bytes, path: Path, header: BhxHeader
) -> dict[str, Any]:
    payload = blob[32:]
    hints: dict[str, Any] = {
        "first_payload_words": [
            f"0x{word:08x}" for word in payload_words(blob, 16)
        ],
        "last_payload_words": [
            f"0x{word:08x}" for word in payload_words_tail(payload, 8)
        ],
        "single_segment_container": True,
        "payload_is_extractable_binary": True,
        "payload_layout": "contiguous 16-bit word image",
    }

    if "pcs" in {part.lower() for part in path.parts}:
        hints["pcs_payload"] = decode_pcs_payload(blob, path)

    hints["target_interpretation"] = {
        "target_hex": f"0x{header.target:08x}",
        "likely_meaning": "load base or region selector",
    }

    c2000_span = decode_f28377d_span(header.target, header.payload_size)
    if c2000_span is not None:
        hints["f28377d_flash_span"] = c2000_span
        pcs_payload = hints.get("pcs_payload")
        if pcs_payload and "word7_c28x_address_candidate" in pcs_payload:
            end_minus_one = int(c2000_span["end"], 16) - 1
            pcs_payload["word7_matches_end_minus_one"] = (
                int(pcs_payload["word7_c28x_address_candidate"], 16)
                == end_minus_one
            )

    c28x_analysis = decode_c28x_image_analysis(payload, header.target)
    if c28x_analysis is not None:
        hints["c28x_image_analysis"] = c28x_analysis

    return hints


def payload_words_tail(payload: bytes, count: int) -> list[int]:
    usable = len(payload) // 4
    start = max(0, usable - count)
    return [read_u32be(payload, index * 4) for index in range(start, usable)]


def extract_payload(path: Path, output_path: Path) -> None:
    blob = path.read_bytes()
    payload = blob[32:]
    output_path.write_bytes(payload)


def parse_bhx(path: Path) -> dict[str, Any]:
    blob = path.read_bytes()
    header = decode_bhx_header(blob)
    return {
        "path": str(path),
        "header": asdict(header),
        "payload_hints": decode_payload_hints(blob, path, header),
    }


def print_text_report(report: dict[str, Any]) -> None:
    header = report["header"]
    print(report["path"])
    print(f"  global_magic: {header['global_magic']}")
    print(f"  global_version: {header['global_version']}")
    print(f"  file_size: {header['file_size']} (0x{header['file_size']:x})")
    print(f"  segment_magic: {header['segment_magic']}")
    print(f"  segment_version: {header['segment_version']}")
    print(f"  target: 0x{header['target']:08x}")
    print(
        "  payload_size_in_header: "
        f"{header['payload_size_in_header']} "
        f"(0x{header['payload_size_in_header']:x})"
    )
    print(
        "  repeated_payload_size: "
        f"{header['repeated_payload_size']} "
        f"(0x{header['repeated_payload_size']:x})"
    )
    print(f"  payload_crc32: 0x{header['payload_crc32']:08x}")
    print(
        f"  payload_size: {header['payload_size']} "
        f"(0x{header['payload_size']:x})"
    )
    print(f"  payload_size_matches: {header['payload_size_matches']}")
    print(
        "  repeated_payload_size_matches: "
        f"{header['repeated_payload_size_matches']}"
    )
    print(f"  payload_crc_matches: {header['payload_crc_matches']}")

    payload_hints = report["payload_hints"]
    payload_words_text = ", ".join(payload_hints["first_payload_words"])
    print(f"  first_payload_words: {payload_words_text}")
    tail_words_text = ", ".join(payload_hints["last_payload_words"])
    print(f"  last_payload_words: {tail_words_text}")
    print(
        "  payload_layout: "
        f"{payload_hints['payload_layout']}"
    )

    pcs_payload = payload_hints.get("pcs_payload")
    if pcs_payload:
        print("  pcs_payload:")
        print(
            "    component_id_candidate: "
            f"0x{pcs_payload['component_id_candidate']:04x}"
        )
        print(f"    a_value_candidate: {pcs_payload['a_value_candidate']}")
        print(f"    p_value_candidate: {pcs_payload['p_value_candidate']}")
        print(f"    u_value_candidate: {pcs_payload['u_value_candidate']}")
        print(f"    filename_hints: {pcs_payload['filename_hints']}")
        if "word6_c28x_address_candidate" in pcs_payload:
            print(
                "    word6_c28x_address_candidate: "
                f"{pcs_payload['word6_c28x_address_candidate']}"
            )
        if "word7_c28x_address_candidate" in pcs_payload:
            print(
                "    word7_c28x_address_candidate: "
                f"{pcs_payload['word7_c28x_address_candidate']}"
            )
        if "word7_matches_end_minus_one" in pcs_payload:
            print(
                "    word7_matches_end_minus_one: "
                f"{pcs_payload['word7_matches_end_minus_one']}"
            )
        if "section_words_between_word6_and_word7" in pcs_payload:
            print(
                "    section_words_between_word6_and_word7: "
                f"{pcs_payload['section_words_between_word6_and_word7']}"
            )
        if "section_bytes_between_word6_and_word7" in pcs_payload:
            print(
                "    section_bytes_between_word6_and_word7: "
                f"{pcs_payload['section_bytes_between_word6_and_word7']}"
            )
        if "tail_penultimate_word" in pcs_payload:
            print(
                "    tail_penultimate_word: "
                f"{pcs_payload['tail_penultimate_word']}"
            )
        if "tail_last_word" in pcs_payload:
            print(
                "    tail_last_word: "
                f"{pcs_payload['tail_last_word']}"
            )

    flash_span = payload_hints.get("f28377d_flash_span")
    if flash_span:
        print("  f28377d_flash_span:")
        print(f"    address_unit: {flash_span['address_unit']}")
        print(f"    start: {flash_span['start']}")
        print(f"    end: {flash_span['end']}")
        print(f"    payload_words_16: {flash_span['payload_words_16']}")
        print(f"    sectors: {', '.join(flash_span['sectors'])}")

    c28x_analysis = payload_hints.get("c28x_image_analysis")
    if c28x_analysis:
        print("  c28x_image_analysis:")
        print(
            "    includes_flash_begin_region: "
            f"{c28x_analysis['includes_flash_begin_region']}"
        )
        print(
            "    likely_flash_vector_table_present: "
            f"{c28x_analysis['likely_flash_vector_table_present']}"
        )
        print(
            "    vector_table_reason: "
            f"{c28x_analysis['vector_table_reason']}"
        )
        print(
            "    flash_address_like_word_count: "
            f"{c28x_analysis['flash_address_like_word_count']}"
        )
        print(
            "    flash_address_like_words_in_first_0x40_bytes: "
            f"{c28x_analysis['flash_address_like_words_in_first_0x40_bytes']}"
        )
        print(
            "    trailing_flash_address_run_count: "
            f"{c28x_analysis['trailing_flash_address_run_count']}"
        )
        print(
            "    trailing_flash_address_stride_words: "
            f"{c28x_analysis['trailing_flash_address_stride_words']}"
        )
        print(
            "    first_executable_offset_bytes_candidate: "
            f"0x{c28x_analysis['first_executable_offset_bytes_candidate']:x}"
        )
        print(
            "    first_executable_address_candidate: "
            f"{c28x_analysis['first_executable_address_candidate']}"
        )
        print(
            "    first_executable_preview_hex: "
            f"{c28x_analysis['first_executable_preview_hex']}"
        )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inspect Tesla BHX firmware containers"
    )
    parser.add_argument("paths", nargs="+", help="BHX files to parse")
    parser.add_argument(
        "--json", action="store_true", help="Emit JSON instead of text"
    )
    parser.add_argument(
        "--extract-dir",
        help="Write extracted payload binaries into this directory",
    )
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    reports = [parse_bhx(Path(path)) for path in args.paths]
    if args.extract_dir:
        extract_dir = Path(args.extract_dir)
        extract_dir.mkdir(parents=True, exist_ok=True)
        for path_str in args.paths:
            source = Path(path_str)
            output = extract_dir / f"{source.stem}.payload.bin"
            extract_payload(source, output)

    if args.json:
        print(json.dumps(reports, indent=2, sort_keys=True))
    else:
        for index, report in enumerate(reports):
            if index:
                print()
            print_text_report(report)
        if args.extract_dir:
            print()
            print(f"Extracted payloads to {args.extract_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
