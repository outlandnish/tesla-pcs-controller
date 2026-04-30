/*
 * SLCAN (LAWICEL) bridge firmware for Tesla PCS Charger hardware.
 *
 * Bridges one CAN bus to USART1 using the LAWICEL ASCII protocol so the
 * device appears as a SocketCAN interface on a Linux host via slcand:
 *
 *   slcand -o -c -s6 /dev/ttyUSB0 can0
 *   ip link set can0 up
 *
 * Protocol summary (terminated with CR, error reply BEL=0x07, ok reply CR):
 *   Sn      set bitrate preset (n=0..8)
 *   sxxyy   set BTR0/BTR1 (not supported, returns BEL)
 *   O       open channel (normal mode)
 *   L       open channel (listen-only)
 *   C       close channel
 *   tiiildd...      send standard frame  (3 hex id, 1 hex dlc, data)
 *   Tiiiiiiiildd... send extended frame  (8 hex id)
 *   riiil           send standard RTR
 *   Riiiiiiiil      send extended RTR
 *   F       read status flags  (returns "Fxx\r")
 *   V       hw version         (returns "Vxxyy\r")
 *   v       sw version         (returns "vxxyy\r")
 *   N       serial number      (returns "Nxxxx\r")
 *   Z0/Z1   timestamps off/on
 *   M/m     acceptance code/mask (accepted, no-op - hardware filter open)
 *
 * Received frames are emitted in the same tiiildd.../Tiiiiiiiildd... format.
 * Timestamps (when enabled) append a 4-hex-digit ms field (0..0xEA5F) before CR.
 */

#include <Arduino.h>
#include "hwdefs.h"
#include "serial_config.h"
#include "can.h"
#include "can_common.h"

HardwareSerial Serial1(USART1);

// Pick which CAN bus to bridge (default: CAN1 = PCS IPC bus).
// Override with -D SLCAN_BUS=2 or 3 in build_flags.
#ifndef SLCAN_BUS
#define SLCAN_BUS 1
#endif

#if SLCAN_BUS == 1
static CANBus slcanBus(PIN_CANRX0, PIN_CANTX0);
#elif SLCAN_BUS == 2
static CANBus slcanBus(PIN_CANRX1, PIN_CANTX1);
#elif SLCAN_BUS == 3
static CANBus slcanBus(PIN_CANRX2, PIN_CANTX2);
#else
#error "SLCAN_BUS must be 1, 2, or 3"
#endif

#ifndef SLCAN_SERIAL_BAUD
#define SLCAN_SERIAL_BAUD 921600
#endif

// Set -D SLCAN_DEBUG=1 to enable boot banner, dbg* logs, and the 'D' diag command.
// When off (default), the serial port emits only protocol bytes - cleaner for
// host-side parsers that don't expect any chatter.
#ifndef SLCAN_DEBUG
#define SLCAN_DEBUG 0
#endif

static constexpr char CR = '\r';
static constexpr char BEL = 0x07;

static constexpr size_t CMD_BUF_SIZE = 64;
static char cmdBuf[CMD_BUF_SIZE];
static size_t cmdLen = 0;

static bool channelOpen = false;
static bool listenOnly = false;
static bool timestampsEnabled = false;
static uint32_t pendingBitrate = CAN_BPS_500K;  // Default S6.

// Stats - readable via 'D' diag command while channel is closed.
static uint32_t rxFrameCount = 0;
static uint32_t txFrameCount = 0;
static uint32_t rxOverflowCount = 0;
static uint32_t cmdErrorCount = 0;
static uint32_t lastRxBlinkMs = 0;
static uint32_t lastTxBlinkMs = 0;

// Debug helpers - only safe to call while channelOpen == false.
// Once SLCAN is open, anything we print would corrupt the binary protocol.
#if SLCAN_DEBUG
static inline void dbgPrint(const char* s) {
  if (!channelOpen) Serial.print(s);
}
static inline void dbgPrintln(const char* s) {
  if (!channelOpen) Serial.println(s);
}
#else
static inline void dbgPrint(const char*) {}
static inline void dbgPrintln(const char*) {}
#endif

// LAWICEL bitrate presets S0..S8. can_common only defines 25K/50K/125K/250K/500K/800K/1000K,
// so the unsupported low rates use raw bps values (the CAN init clamps/configures from bps).
static const uint32_t bitrateTable[9] = {
  10000,
  20000,
  CAN_BPS_50K,
  100000,
  CAN_BPS_125K,
  CAN_BPS_250K,
  CAN_BPS_500K,
  CAN_BPS_800K,
  CAN_BPS_1000K,
};

static inline int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static bool parseHex(const char* s, size_t n, uint32_t& out) {
  out = 0;
  for (size_t i = 0; i < n; i++) {
    int v = hexVal(s[i]);
    if (v < 0) return false;
    out = (out << 4) | (uint32_t)v;
  }
  return true;
}

static inline void writeHexNibble(uint8_t v) {
  Serial.write("0123456789ABCDEF"[v & 0x0F]);
}

static inline void writeHexByte(uint8_t v) {
  writeHexNibble(v >> 4);
  writeHexNibble(v);
}

static void writeHex(uint32_t v, uint8_t nibbles) {
  for (int8_t i = nibbles - 1; i >= 0; i--) {
    writeHexNibble((uint8_t)(v >> (i * 4)));
  }
}

static void replyOk() { Serial.write(CR); }
static void replyErr() { cmdErrorCount++; Serial.write(BEL); }

static bool openChannel(bool listen) {
  if (channelOpen) return false;
#if SLCAN_DEBUG
  dbgPrint("[slcan] opening bitrate=");
  Serial.print(pendingBitrate);
  dbgPrintln(listen ? " mode=listen" : " mode=normal");
#endif
  if (!slcanBus.begin(pendingBitrate)) {
    dbgPrintln("[slcan] CAN init FAILED");
    return false;
  }
  slcanBus.setListenOnlyMode(listen);
  // Open acceptance filter: accept all standard and extended frames.
  slcanBus.setFilter(0, 0, 0);
  channelOpen = true;
  listenOnly = listen;
  // Power up the PCS only while the host is actively bridged.
  digitalWrite(PIN_PCS_ENABLE, HIGH);
  return true;
}

static void closeChannel() {
  if (!channelOpen) return;
  slcanBus.end();
  channelOpen = false;
  listenOnly = false;
  // Drop PCS_ENABLE so the PCS stops driving the IPC bus when nobody is listening.
  digitalWrite(PIN_PCS_ENABLE, LOW);
  // Also release charge/DCDC enables (active-low → HIGH = off).
  digitalWrite(PIN_CHARGE_ENABLE, HIGH);
  digitalWrite(PIN_DCDC_ENABLE, HIGH);
}

// Snoop host-originated TX frames for the PCS mode signal.
// Message 0x22A byte[2] low nibble (current_mode) carries:
//   0x0 OFF, 0x5 CHARGE_ONLY, 0x9 DCDC_ONLY, 0xD CHARGE+DCDC
// Bit 2 (0x4) -> charge active, bit 3 (0x8) -> DCDC ("HV") active.
// Both enable pins are active-low.
static void applyEnablesFromTxFrame(const CAN_FRAME& f) {
  if (f.extended || f.rtr) return;
  if (f.id != 0x22A || f.length < 3) return;
  uint8_t mode = f.data.uint8[2] & 0x0F;
  bool chargeOn = (mode & 0x04) != 0;
  bool dcdcOn   = (mode & 0x08) != 0;
  digitalWrite(PIN_CHARGE_ENABLE, chargeOn ? LOW : HIGH);
  digitalWrite(PIN_DCDC_ENABLE,   dcdcOn   ? LOW : HIGH);
}

// Parse and send a tx command. Returns true on success.
// kind: 't'=std data, 'T'=ext data, 'r'=std rtr, 'R'=ext rtr
static bool handleTxCommand(char kind, const char* args, size_t argLen) {
  bool extended = (kind == 'T' || kind == 'R');
  bool rtr = (kind == 'r' || kind == 'R');
  size_t idChars = extended ? 8 : 3;

  if (argLen < idChars + 1) return false;

  uint32_t id;
  if (!parseHex(args, idChars, id)) return false;

  int dlc = hexVal(args[idChars]);
  if (dlc < 0 || dlc > 8) return false;

  size_t dataChars = rtr ? 0 : (size_t)dlc * 2;
  if (argLen < idChars + 1 + dataChars) return false;

  CAN_FRAME frame;
  frame.id = id;
  frame.extended = extended ? 1 : 0;
  frame.rtr = rtr ? 1 : 0;
  frame.length = (uint8_t)dlc;
  frame.priority = 0;
  for (int i = 0; i < 8; i++) frame.data.uint8[i] = 0;

  for (int i = 0; i < dlc && !rtr; i++) {
    int hi = hexVal(args[idChars + 1 + i * 2]);
    int lo = hexVal(args[idChars + 1 + i * 2 + 1]);
    if (hi < 0 || lo < 0) return false;
    frame.data.uint8[i] = (uint8_t)((hi << 4) | lo);
  }

  if (!channelOpen || listenOnly) return false;
  bool ok = slcanBus.sendFrame(frame);
  if (ok) {
    txFrameCount++;
    lastTxBlinkMs = millis();
    applyEnablesFromTxFrame(frame);
  }
  return ok;
}

static void emitRxFrame(const CAN_FRAME& f) {
  char prefix;
  if (f.extended) prefix = f.rtr ? 'R' : 'T';
  else            prefix = f.rtr ? 'r' : 't';

  Serial.write(prefix);
  writeHex(f.id, f.extended ? 8 : 3);
  writeHexNibble(f.length & 0x0F);
  if (!f.rtr) {
    for (uint8_t i = 0; i < f.length; i++) writeHexByte(f.data.uint8[i]);
  }
  if (timestampsEnabled) {
    // LAWICEL timestamp: ms counter mod 0xEA60 (60000), 4 hex digits.
    uint16_t ts = (uint16_t)(millis() % 0xEA60);
    writeHex(ts, 4);
  }
  Serial.write(CR);
}

static void processCommand() {
  if (cmdLen == 0) return;
  char cmd = cmdBuf[0];
  const char* args = &cmdBuf[1];
  size_t argLen = cmdLen - 1;

  switch (cmd) {
    case 'S': {
      if (argLen != 1 || channelOpen) { replyErr(); return; }
      int n = cmdBuf[1] - '0';
      if (n < 0 || n > 8) { replyErr(); return; }
      pendingBitrate = bitrateTable[n];
      replyOk();
      break;
    }
    case 's':
      // BTR0/BTR1 raw register set - not supported.
      replyErr();
      break;
    case 'O':
      if (openChannel(false)) replyOk(); else replyErr();
      break;
    case 'L':
      if (openChannel(true)) replyOk(); else replyErr();
      break;
    case 'C':
      closeChannel();
      replyOk();
      break;
    case 't': case 'T': case 'r': case 'R':
      if (handleTxCommand(cmd, args, argLen)) {
        // Lowercase ack adds 'z\r', uppercase 'Z\r' per LAWICEL spec.
        Serial.write((cmd == 't' || cmd == 'r') ? 'z' : 'Z');
        replyOk();
      } else {
        replyErr();
      }
      break;
    case 'F':
      Serial.write('F');
      writeHexByte(0x00);  // No errors flagged.
      replyOk();
      break;
    case 'V':
      Serial.write("V0101");
      replyOk();
      break;
    case 'v':
      Serial.write("v0100");
      replyOk();
      break;
    case 'N':
      Serial.write("NPCS1");
      replyOk();
      break;
    case 'Z':
      if (argLen != 1) { replyErr(); return; }
      if (cmdBuf[1] == '0') { timestampsEnabled = false; replyOk(); }
      else if (cmdBuf[1] == '1') { timestampsEnabled = true; replyOk(); }
      else replyErr();
      break;
    case 'M': case 'm':
      // Acceptance code/mask - accepted but ignored (filter is fully open).
      replyOk();
      break;
#if SLCAN_DEBUG
    case 'D':
      // Non-standard diagnostic command - only useful while channel is closed
      // (would corrupt the binary protocol once open). Returns multi-line status.
      if (channelOpen) { replyErr(); return; }
      Serial.println();
      Serial.println("=== SLCAN bridge status ===");
      Serial.print("  bus            = CAN"); Serial.println(SLCAN_BUS);
      Serial.print("  serial baud    = "); Serial.println(SLCAN_SERIAL_BAUD);
      Serial.print("  pending bps    = "); Serial.println(pendingBitrate);
      Serial.print("  channel open   = "); Serial.println(channelOpen ? "yes" : "no");
      Serial.print("  listen-only    = "); Serial.println(listenOnly ? "yes" : "no");
      Serial.print("  timestamps     = "); Serial.println(timestampsEnabled ? "on" : "off");
      Serial.print("  rx frames      = "); Serial.println(rxFrameCount);
      Serial.print("  tx frames      = "); Serial.println(txFrameCount);
      Serial.print("  rx overflows   = "); Serial.println(rxOverflowCount);
      Serial.print("  cmd errors     = "); Serial.println(cmdErrorCount);
      Serial.print("  uptime ms      = "); Serial.println(millis());
      Serial.println("===========================");
      replyOk();
      break;
#endif
    default:
      replyErr();
      break;
  }
}

static void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == CR) {
      processCommand();
      cmdLen = 0;
    } else if (c == '\n' || c == 0x08 || c == 0x7F) {
      // Ignore LF / BS / DEL (terminal backspace).
    } else {
      if (cmdLen < CMD_BUF_SIZE - 1) {
        cmdBuf[cmdLen++] = c;
      } else {
        // Overflow - reset and signal error.
        cmdLen = 0;
        replyErr();
      }
    }
  }
}

static void pollCan() {
  if (!channelOpen) return;
  CAN_FRAME f;
  uint8_t drained = 0;
  // Drain RX FIFO to avoid overflow at high bus loads.
  for (int i = 0; i < 16; i++) {
    if (!slcanBus.get_rx_buff(f)) break;
    emitRxFrame(f);
    rxFrameCount++;
    drained++;
  }
  if (drained > 0) lastRxBlinkMs = millis();
  // FIFO holds 3 messages on STM32; if we drained all 16 in one pass,
  // we likely missed some - flag it so the host can see it via 'D'.
  if (drained == 16) rxOverflowCount++;
}

void setup() {
  Serial.setRx(PB7_ALT0);
  Serial.setTx(PB6_ALT1);
  Serial.begin(SLCAN_SERIAL_BAUD);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // PCS_ENABLE stays low at boot - asserted only while the SLCAN channel is open.
  // Charge/DCDC enables stay deasserted (active-low → HIGH); host issues control
  // via CAN once the bridge is up.
  pinMode(PIN_PCS_ENABLE, OUTPUT);
  pinMode(PIN_CHARGE_ENABLE, OUTPUT);
  pinMode(PIN_DCDC_ENABLE, OUTPUT);
  digitalWrite(PIN_CHARGE_ENABLE, HIGH);
  digitalWrite(PIN_DCDC_ENABLE, HIGH);
  digitalWrite(PIN_PCS_ENABLE, LOW);

#if SLCAN_DEBUG
  // Boot banner - safe because channelOpen is false. Sent at SLCAN_SERIAL_BAUD,
  // so a host terminal at the wrong rate will show garbage and you'll know.
  Serial.println();
  Serial.println("==========================================");
  Serial.println(" Tesla PCS - SLCAN bridge firmware");
  Serial.print  ("  bus         : CAN"); Serial.println(SLCAN_BUS);
  Serial.print  ("  serial baud : "); Serial.println(SLCAN_SERIAL_BAUD);
  Serial.println("  default bps : 500000 (S6)");
  Serial.println("  PCS_ENABLE  : asserted on O/L, released on C");
  Serial.println("");
  Serial.println(" Send Sn<CR> O<CR> to open. Send D<CR> for diag.");
  Serial.println(" After O<CR>, output becomes binary SLCAN.");
  Serial.println("==========================================");
#endif
}

void loop() {
  pollSerial();
  pollCan();

  // LED feedback:
  //   channel closed         -> solid off, brief flash every 2s as alive heartbeat
  //   channel open, idle     -> solid on
  //   channel open + RX/TX   -> brief blink off for 50ms on each event
  uint32_t now = millis();
  if (!channelOpen) {
    static uint32_t lastAlive = 0;
    if (now - lastAlive >= 2000) {
      lastAlive = now;
      digitalWrite(PIN_LED, HIGH);
    } else if (now - lastAlive >= 50) {
      digitalWrite(PIN_LED, LOW);
    }
  } else {
    bool active = (now - lastRxBlinkMs < 50) || (now - lastTxBlinkMs < 50);
    digitalWrite(PIN_LED, active ? LOW : HIGH);
  }
}
