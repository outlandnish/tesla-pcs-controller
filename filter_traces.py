#!/usr/bin/env python3
"""Filter CAN trace files by message ID."""

import sys
import re
import argparse


def parse_trace_line(line):
    """Parse a single trace line.

    Returns dict with timestamp, msg_type, can_id, length, data_bytes or None.
    """
    # Skip comments and empty lines
    if line.strip().startswith(';') or not line.strip():
        return None

    # Match pattern: number) timestamp Type CAN_ID Length Data
    # Example: 1)     50601.4  Rx         0272  8  06 00 04 C1 07 C0 1E 20
    pattern = r'\s*\d+\)\s+([\d.]+)\s+(\w+)\s+([0-9A-Fa-f]+)\s+(\d+)\s+(.*)'
    match = re.match(pattern, line)

    if not match:
        return None

    timestamp = float(match.group(1))
    msg_type = match.group(2)
    can_id = match.group(3).upper().zfill(4)
    length = int(match.group(4))
    data_str = match.group(5).strip()

    # Parse data bytes
    data_bytes = data_str.split() if data_str else []

    return {
        'timestamp': timestamp,
        'type': msg_type,
        'id': can_id,
        'length': length,
        'data': data_bytes,
        'line': line.rstrip('\n')
    }


def normalize_can_id(can_id):
    """Normalize CAN ID to 4-digit hex format.

    Accepts hex (0x prefix) or decimal IDs.
    Examples: '0x545', '545' (decimal 1349 = hex 545), '13D' (hex)
    """
    can_id = can_id.strip()
    if can_id.lower().startswith('0x'):
        # Hex format
        can_id = can_id[2:]
    else:
        # Try to parse as decimal, convert to hex
        try:
            decimal_val = int(can_id)
            can_id = format(decimal_val, 'x')
        except ValueError:
            # Not decimal, assume hex without 0x prefix
            pass
    return can_id.upper().zfill(4)


def filter_traces(input_file, can_ids, output_file=None):
    """Filter trace file by CAN IDs."""
    # Normalize CAN IDs to 4-digit hex format (e.g., '545' -> '0545')
    target_ids = set()
    for can_id in can_ids:
        target_ids.add(normalize_can_id(can_id))

    print(f"Filtering for CAN IDs: {sorted(target_ids)}")

    matched_count = 0
    line_count = 0

    try:
        outfile = open(output_file, 'w') if output_file else sys.stdout
        with open(input_file, 'r') as infile:
            # Skip header comments and process data lines
            for line in infile:
                parsed = parse_trace_line(line)
                if parsed:
                    line_count += 1
                    if parsed['id'] in target_ids:
                        matched_count += 1
                        ts = parsed['timestamp']
                        cid = parsed['id'].lstrip('0') or '0'
                        dlc = parsed['length']
                        data = ' '.join(parsed['data'])
                        outfile.write(
                            f"{ts:.1f},0x{cid},{dlc},{data}\n"
                        )

        if output_file:
            outfile.close()

        print(f"\nResults:")
        print(f"  Input lines processed: {line_count}")
        print(f"  Matched messages: {matched_count}")
        if output_file:
            print(f"  Output file: {output_file}")

    except FileNotFoundError:
        print(f"Error: Input file '{input_file}' not found", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description='Filter CAN trace files by message ID',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Output to stdout
  python filter_traces.py -i trace.trc -f 545,13D,20A

  # Output to file
  python filter_traces.py -i trace.trc -f 545,13D,20A -o filtered.trc

  # Works with 0x prefix and leading zeros
  python filter_traces.py -i trace.trc -f 0x545,0x13D,0x20A
        '''
    )
    parser.add_argument(
        '-i', '--input',
        required=True,
        help='Input trace file'
    )
    parser.add_argument(
        '-f', '--filters',
        required=True,
        help='Comma-separated list of CAN IDs to filter (e.g., 545,13D,0x20A)'
    )
    parser.add_argument(
        '-o', '--output',
        default=None,
        help='Output file (default: stdout)'
    )

    args = parser.parse_args()

    # Parse comma-separated CAN IDs
    can_ids = [id.strip() for id in args.filters.split(',')]

    filter_traces(args.input, can_ids, args.output)


if __name__ == '__main__':
    main()
