# PCS Alert Resolution: Missing VCFRONT CAN Messages Plan

**Date:** 2026-03-31
**Branch:** feature/dbc-can-updates
**Goal:** Resolve PCS_a024_vcfrontMia, PCS_a107_vcPcsDCDCInterfaceMia, PCS_a067_dcdcLvRationality

## Background

Cross-referenced the real Tesla Model 3 charging trace (`data/traces/charging initiation.trc`,
~44 seconds, 100k messages, 190 unique IDs) against the compact DBC (`data/Model3_ETH.compact.json`)
and the alt DBC (`data/model3_alt.dbc`) to identify messages the PCS expects but our firmware does
not send.

---

## New Candidate VCFRONT Messages From Trace + Enum Maps

New filtered captures (`charge-vc-pcs-441.csv`, `charge-vc-pcs-443.csv`) plus enum tables from
`can_frames_decoded_enum_values_mcu3.csv` show two additional VC-origin messages that likely
participate in PCS interface health checks.

### 0x441 — VC_pcsInterface

**Observed in trace:** 8 bytes, ~50ms cycle, alternating two payload templates.

**Enum hints:**

- `VC_pcsInterfaceMuxIndex`: values 0, 1
- `VC_pcsResistanceFiltered`: `0xFFFF = SNA`

**Observed payload templates:**

- Template A: `3A 85 89 00 32 00 [idx] [chk]`
  - `idx` cycles: `0x00, 0x08, 0x10, ... 0x38` (upper nibble appears to be a 4-bit counter)
  - `chk` follows `chk = idx + 0xE3` (mod 256)
- Template B: `[hdr] 01 B6 C3 58 00 [idx] [chk]`
  - `hdr` seen as `0x72/0x73/0x74/0x75` (slowly varying)
  - `idx` cycles: `0x05, 0x0D, 0x15, ... 0x3D` (same 4-bit counter pattern, mux bit set)
  - `chk` follows `chk = idx + K` where `K` tracks `hdr` (`0xAD..0xB0` in this capture)

**Working draft definition (confidence: medium):**

- Byte 6 upper nibble: alive counter (0..15)
- Byte 6 bit 0: mux page select (`0`/`1`), matching enum `VC_pcsInterfaceMuxIndex`
- Mux 0 payload matches template A constants
- Mux 1 payload matches template B constants
- Byte 7: checksum/counter-companion byte (exact algorithm unknown, but deterministic)

### 0x443 — VC_pcsManagement

**Observed in trace:** 6 bytes, ~500ms cycle.

**Enum hints:**

- `VC_pcsManagementMuxIndex`: `INDEX_0`
- `VC_dcdcLVVoltageTargetMax`: `0x3FF = UNRESTRICTED`
- `VC_dcdcLVVoltageTargetMin`: `0x000 = UNRESTRICTED`
- `VC_dcdcLVInputPowerLimit`: `0xFF = UNRESTRICTED`

**Observed payload pattern:**

- `[B0] [B1] FF 03 00 00`
- `B1` increments `0x00..0x0F`
- `B0 = 0x6B + B1` (so range `0x6B..0x7A`)

**Working draft definition (confidence: high for constants):**

- Byte 2-3 encode `0x03FF` (`dcdcLVVoltageTargetMax = UNRESTRICTED`)
- Byte 4-5 encode `0x0000` (`dcdcLVVoltageTargetMin = UNRESTRICTED`)
- Byte 2 low byte `0xFF` is also consistent with `dcdcLVInputPowerLimit = UNRESTRICTED`
- Byte 0/1 carry alive/mux metadata (exact bit split TBD)

### Immediate Firmware Strategy

1. Add TX skeleton for 0x441 at 50ms using the two observed templates and a 4-bit counter.
2. Add TX skeleton for 0x443 at 500ms using fixed bytes `FF 03 00 00` and rolling metadata bytes.
3. Use exact captured patterns first (trace-faithful), then refine bit-level signal mapping later.

These two messages are strong candidates for reducing:

- `PCS_a024_vcFrontMia`
- `PCS_a107_vcPcsDCDCInterfaceMia`
