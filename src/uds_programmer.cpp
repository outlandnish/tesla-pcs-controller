/*
 * UDS Programmer bridge
 *
 * Receives simple ASCII commands over serial and performs UDS programming
 * on CAN ID 0x628/0x629.
 *
 * Serial protocol:
 *   PING
 *   PREP
 *   START <addr_hex> <size_hex> [memId_hex] [alfi_hex]
 *   CHUNK <hex_bytes>
 *   END
 *   ABORT
 *
 * Replies:
 *   OK ...
 *   ERR ...
 */

#include <Arduino.h>
#include <cstring>
#include <cstdlib>
#include "hwdefs.h"
#include "serial_config.h"
#include "debug_serial.h"
#include "../lib/can/can.h"

HardwareSerial Serial1(USART1);
CANBus ipcCan(PIN_CANRX0, PIN_CANTX0);

#define UDS_REQUEST_ID  0x628
#define UDS_RESPONSE_ID 0x629

#define UDS_DIAG_SESSION_CONTROL    0x10
#define UDS_SECURITY_ACCESS         0x27
#define UDS_REQUEST_DOWNLOAD        0x34
#define UDS_TRANSFER_DATA           0x36
#define UDS_TRANSFER_EXIT           0x37
#define UDS_TESTER_PRESENT          0x3E

#define DIAG_SESSION_PROGRAMMING    0x02

#define MAX_RESPONSE_LEN 256
#define MAX_LINE_LEN 600
#define MAX_CHUNK_BYTES 256

static uint8_t response_buf[MAX_RESPONSE_LEN];
static uint16_t response_len = 0;
static bool response_valid = false;

struct ProgramState {
    bool prepared = false;
    bool started = false;
    uint32_t addr = 0;
    uint32_t total_size = 0;
    uint32_t sent = 0;
    uint8_t block_counter = 1;
    uint8_t mem_id = 0;
    uint8_t alfi = 0x44;
    uint8_t addr_len = 4;
    uint8_t size_len = 4;
};

static ProgramState g_state;
static char line_buf[MAX_LINE_LEN];
static size_t line_len = 0;

static void print_hex(const uint8_t* data, int len) {
    for (int i = 0; i < len; i++) {
        if (data[i] < 0x10) DEBUG_SERIAL.print("0");
        DEBUG_SERIAL.print(data[i], HEX);
        DEBUG_SERIAL.print(" ");
    }
}

static void send_uds_single(const uint8_t* payload, int payload_len) {
    uint8_t data[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
    data[0] = payload_len;
    int n = (payload_len > 7) ? 7 : payload_len;
    memcpy(&data[1], payload, n);
    ipcCan.sendMessage(UDS_REQUEST_ID, data, 8);
}

static bool wait_flow_control(uint16_t timeout_ms) {
    unsigned long start = millis();
    while ((millis() - start) < timeout_ms) {
        uint32_t can_id;
        uint8_t data[8];
        uint8_t len;
        if (!ipcCan.receiveMessage(can_id, data, len)) {
            delay(1);
            continue;
        }
        if (can_id == UDS_RESPONSE_ID && ((data[0] >> 4) == 3)) {
            return true;
        }
    }
    return false;
}

static bool send_uds_multi(const uint8_t* payload, int payload_len) {
    if (payload_len <= 7) {
        send_uds_single(payload, payload_len);
        return true;
    }

    uint8_t ff[8];
    ff[0] = 0x10 | ((payload_len >> 8) & 0x0F);
    ff[1] = payload_len & 0xFF;
    int ff_data = (payload_len < 6) ? payload_len : 6;
    memcpy(&ff[2], payload, ff_data);
    for (int i = ff_data + 2; i < 8; i++) ff[i] = 0xAA;
    ipcCan.sendMessage(UDS_REQUEST_ID, ff, 8);

    if (!wait_flow_control(1000)) {
        return false;
    }

    int offset = 6;
    uint8_t seq = 1;
    while (offset < payload_len) {
        uint8_t cf[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
        cf[0] = 0x20 | (seq & 0x0F);
        int remaining = payload_len - offset;
        int cf_data = (remaining < 7) ? remaining : 7;
        memcpy(&cf[1], &payload[offset], cf_data);
        ipcCan.sendMessage(UDS_REQUEST_ID, cf, 8);

        offset += cf_data;
        seq = (seq + 1) & 0x0F;
        delay(5);
    }
    return true;
}

static void send_flow_control() {
    uint8_t fc[8] = {0x30, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
    ipcCan.sendMessage(UDS_REQUEST_ID, fc, 8);
}

static bool wait_response(int timeout_ms) {
    unsigned long start = millis();
    response_valid = false;
    response_len = 0;

    while ((millis() - start) < (unsigned long)timeout_ms) {
        uint32_t can_id;
        uint8_t data[8];
        uint8_t len;

        if (!ipcCan.receiveMessage(can_id, data, len)) {
            delay(1);
            continue;
        }
        if (can_id != UDS_RESPONSE_ID) continue;

        uint8_t pci_type = (data[0] >> 4) & 0x0F;
        if (pci_type == 0) {
            uint8_t sf_len = data[0] & 0x0F;
            if (sf_len > 7) sf_len = 7;
            memcpy(response_buf, &data[1], sf_len);
            response_len = sf_len;
            response_valid = true;
            return true;
        }

        if (pci_type == 1) {
            uint16_t total_len = ((uint16_t)(data[0] & 0x0F) << 8) | data[1];
            if (total_len > MAX_RESPONSE_LEN) total_len = MAX_RESPONSE_LEN;

            uint16_t copy_len = (total_len < 6) ? total_len : 6;
            memcpy(response_buf, &data[2], copy_len);
            response_len = copy_len;
            send_flow_control();

            unsigned long cf_start = millis();
            while (response_len < total_len && (millis() - cf_start) < 2000) {
                if (!ipcCan.receiveMessage(can_id, data, len)) {
                    delay(1);
                    continue;
                }
                if (can_id != UDS_RESPONSE_ID) continue;
                if ((data[0] >> 4) != 2) continue;

                uint16_t remaining = total_len - response_len;
                uint16_t cf_copy = (remaining < 7) ? remaining : 7;
                memcpy(&response_buf[response_len], &data[1], cf_copy);
                response_len += cf_copy;
            }

            response_valid = true;
            return true;
        }
    }

    return false;
}

static bool is_positive() {
    return response_valid && response_len >= 1 &&
           response_buf[0] >= 0x40 && response_buf[0] < 0x80 &&
           response_buf[0] != 0x7F;
}

static uint8_t get_nrc() {
    if (!response_valid || response_len < 3 || response_buf[0] != 0x7F) return 0;
    return response_buf[2];
}

static bool parse_hex_u32(const char* s, uint32_t* out) {
    if (s == nullptr || *s == '\0') return false;
    char* end = nullptr;
    unsigned long val = strtoul(s, &end, 16);
    if (end == s || *end != '\0') return false;
    *out = (uint32_t)val;
    return true;
}

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool parse_hex_bytes(const char* s, uint8_t* out, int* out_len) {
    int n = (int)strlen(s);
    if ((n == 0) || ((n % 2) != 0)) return false;
    if ((n / 2) > MAX_CHUNK_BYTES) return false;

    int idx = 0;
    for (int i = 0; i < n; i += 2) {
        int hi = hex_nibble(s[i]);
        int lo = hex_nibble(s[i + 1]);
        if (hi < 0 || lo < 0) return false;
        out[idx++] = (uint8_t)((hi << 4) | lo);
    }

    *out_len = idx;
    return true;
}

static bool uds_enter_programming_session() {
    uint8_t req[] = {UDS_DIAG_SESSION_CONTROL, DIAG_SESSION_PROGRAMMING};
    send_uds_single(req, 2);
    return wait_response(1000) && is_positive();
}

static bool uds_security_unlock() {
    uint8_t seed_req[] = {UDS_SECURITY_ACCESS, 0x01};
    send_uds_single(seed_req, 2);
    if (!wait_response(2000) || !is_positive() || response_len < 3) return false;

    uint8_t seed_len = response_len - 2;
    if (seed_len > 16) seed_len = 16;
    uint8_t key[16];
    for (uint8_t i = 0; i < seed_len; i++) {
        key[i] = response_buf[2 + i] ^ 0x35;
    }

    uint8_t key_req[18];
    key_req[0] = UDS_SECURITY_ACCESS;
    key_req[1] = 0x02;
    memcpy(&key_req[2], key, seed_len);

    if (!send_uds_multi(key_req, 2 + seed_len)) return false;
    return wait_response(2000) && is_positive();
}

static bool uds_tester_present() {
    uint8_t tp[] = {UDS_TESTER_PRESENT, 0x00};
    send_uds_single(tp, 2);
    return wait_response(1000) && is_positive();
}

static bool uds_request_download(uint32_t addr, uint32_t size, uint8_t mem_id, uint8_t alfi, uint8_t addr_len, uint8_t size_len) {
    uint8_t req[20];
    int req_len = 0;
    req[req_len++] = UDS_REQUEST_DOWNLOAD;
    req[req_len++] = 0x00;  // no compression/encryption
    req[req_len++] = alfi;

    if (addr_len == 5) {
        req[req_len++] = mem_id;
        req[req_len++] = (uint8_t)((addr >> 24) & 0xFF);
        req[req_len++] = (uint8_t)((addr >> 16) & 0xFF);
        req[req_len++] = (uint8_t)((addr >> 8) & 0xFF);
        req[req_len++] = (uint8_t)(addr & 0xFF);
    } else {
        for (int b = addr_len - 1; b >= 0; b--) {
            req[req_len++] = (uint8_t)((addr >> (8 * b)) & 0xFF);
        }
    }

    for (int b = size_len - 1; b >= 0; b--) {
        req[req_len++] = (uint8_t)((size >> (8 * b)) & 0xFF);
    }

    if (!send_uds_multi(req, req_len)) return false;
    return wait_response(2000) && is_positive();
}

static bool uds_transfer_data(uint8_t block_counter, const uint8_t* data, uint16_t len) {
    uint8_t req[2 + MAX_CHUNK_BYTES];
    req[0] = UDS_TRANSFER_DATA;
    req[1] = block_counter;
    memcpy(&req[2], data, len);

    if (!send_uds_multi(req, 2 + len)) return false;
    if (!wait_response(2000) || !is_positive()) return false;

    if (response_len >= 2 && response_buf[0] == (UDS_TRANSFER_DATA + 0x40)) {
        return response_buf[1] == block_counter;
    }

    return true;
}

static bool uds_transfer_exit() {
    uint8_t req[] = {UDS_TRANSFER_EXIT};
    send_uds_single(req, 1);
    return wait_response(2000) && is_positive();
}

static void reset_program_state() {
    g_state.started = false;
    g_state.addr = 0;
    g_state.total_size = 0;
    g_state.sent = 0;
    g_state.block_counter = 1;
    g_state.mem_id = 0;
    g_state.alfi = 0x44;
    g_state.addr_len = 4;
    g_state.size_len = 4;
}

static void handle_cmd_prepare() {
    if (!uds_enter_programming_session()) {
        DEBUG_SERIAL.print("ERR PREP session nrc=");
        DEBUG_SERIAL.println(get_nrc(), HEX);
        return;
    }
    if (!uds_security_unlock()) {
        DEBUG_SERIAL.print("ERR PREP unlock nrc=");
        DEBUG_SERIAL.println(get_nrc(), HEX);
        return;
    }
    g_state.prepared = true;
    DEBUG_SERIAL.println("OK PREP");
}

static void handle_cmd_start(char* args) {
    char* tok_addr = strtok(args, " ");
    char* tok_size = strtok(nullptr, " ");
    char* tok_mem = strtok(nullptr, " ");
    char* tok_alfi = strtok(nullptr, " ");

    if (tok_addr == nullptr || tok_size == nullptr) {
        DEBUG_SERIAL.println("ERR START usage");
        return;
    }
    if (!g_state.prepared) {
        DEBUG_SERIAL.println("ERR START not_prepared");
        return;
    }

    uint32_t addr = 0;
    uint32_t size = 0;
    if (!parse_hex_u32(tok_addr, &addr) || !parse_hex_u32(tok_size, &size)) {
        DEBUG_SERIAL.println("ERR START parse");
        return;
    }

    uint32_t mem_id = 0;
    if (tok_mem != nullptr && !parse_hex_u32(tok_mem, &mem_id)) {
        DEBUG_SERIAL.println("ERR START memid_parse");
        return;
    }

    uint32_t alfi = 0;
    bool alfi_set = false;
    if (tok_alfi != nullptr) {
        if (!parse_hex_u32(tok_alfi, &alfi)) {
            DEBUG_SERIAL.println("ERR START alfi_parse");
            return;
        }
        alfi_set = true;
    }

    reset_program_state();
    g_state.prepared = true;
    g_state.addr = addr;
    g_state.total_size = size;
    g_state.mem_id = (uint8_t)mem_id;

    if (alfi_set) {
        g_state.alfi = (uint8_t)alfi;
        g_state.addr_len = (uint8_t)(g_state.alfi & 0x0F);
        g_state.size_len = (uint8_t)((g_state.alfi >> 4) & 0x0F);
    } else {
        if (tok_mem != nullptr) {
            g_state.alfi = 0x45;
            g_state.addr_len = 5;
            g_state.size_len = 4;
        } else {
            g_state.alfi = 0x44;
            g_state.addr_len = 4;
            g_state.size_len = 4;
        }
    }

    if (g_state.addr_len < 1 || g_state.addr_len > 5 || g_state.size_len < 1 || g_state.size_len > 4) {
        DEBUG_SERIAL.println("ERR START alfi_range");
        return;
    }

    if (!uds_tester_present()) {
        DEBUG_SERIAL.print("ERR START tester_present nrc=");
        DEBUG_SERIAL.println(get_nrc(), HEX);
        return;
    }

    bool ok = uds_request_download(g_state.addr, g_state.total_size, g_state.mem_id,
                                   g_state.alfi, g_state.addr_len, g_state.size_len);
    if (!ok) {
        DEBUG_SERIAL.print("ERR START download nrc=");
        DEBUG_SERIAL.println(get_nrc(), HEX);
        return;
    }

    g_state.started = true;
    DEBUG_SERIAL.print("OK START addr=0x");
    DEBUG_SERIAL.print(g_state.addr, HEX);
    DEBUG_SERIAL.print(" size=0x");
    DEBUG_SERIAL.print(g_state.total_size, HEX);
    DEBUG_SERIAL.print(" alfi=0x");
    DEBUG_SERIAL.println(g_state.alfi, HEX);
}

static void handle_cmd_chunk(char* args) {
    if (!g_state.started) {
        DEBUG_SERIAL.println("ERR CHUNK not_started");
        return;
    }

    uint8_t chunk[MAX_CHUNK_BYTES];
    int chunk_len = 0;
    if (args == nullptr || !parse_hex_bytes(args, chunk, &chunk_len)) {
        DEBUG_SERIAL.println("ERR CHUNK parse");
        return;
    }

    if ((g_state.sent + (uint32_t)chunk_len) > g_state.total_size) {
        DEBUG_SERIAL.println("ERR CHUNK size_overflow");
        return;
    }

    bool ok = uds_transfer_data(g_state.block_counter, chunk, (uint16_t)chunk_len);
    if (!ok) {
        DEBUG_SERIAL.print("ERR CHUNK transfer nrc=");
        DEBUG_SERIAL.println(get_nrc(), HEX);
        return;
    }

    g_state.sent += (uint32_t)chunk_len;
    g_state.block_counter++;

    DEBUG_SERIAL.print("OK CHUNK sent=");
    DEBUG_SERIAL.print(g_state.sent);
    DEBUG_SERIAL.print("/");
    DEBUG_SERIAL.println(g_state.total_size);
}

static void handle_cmd_end() {
    if (!g_state.started) {
        DEBUG_SERIAL.println("ERR END not_started");
        return;
    }
    if (g_state.sent != g_state.total_size) {
        DEBUG_SERIAL.print("ERR END size_mismatch sent=");
        DEBUG_SERIAL.print(g_state.sent);
        DEBUG_SERIAL.print(" expected=");
        DEBUG_SERIAL.println(g_state.total_size);
        return;
    }

    if (!uds_transfer_exit()) {
        DEBUG_SERIAL.print("ERR END transfer_exit nrc=");
        DEBUG_SERIAL.println(get_nrc(), HEX);
        return;
    }

    DEBUG_SERIAL.println("OK END");
    reset_program_state();
    g_state.prepared = true;
}

static void process_line(char* line) {
    while (*line == ' ') line++;
    if (*line == '\0') return;

    char* cmd = strtok(line, " ");
    char* args = strtok(nullptr, "");
    if (cmd == nullptr) return;

    if (strcmp(cmd, "PING") == 0) {
        DEBUG_SERIAL.println("OK PONG");
        return;
    }
    if (strcmp(cmd, "HELP") == 0) {
        DEBUG_SERIAL.println("OK CMDS PING PREP START CHUNK END ABORT");
        return;
    }
    if (strcmp(cmd, "ABORT") == 0) {
        reset_program_state();
        DEBUG_SERIAL.println("OK ABORT");
        return;
    }
    if (strcmp(cmd, "PREP") == 0) {
        handle_cmd_prepare();
        return;
    }
    if (strcmp(cmd, "START") == 0) {
        handle_cmd_start(args);
        return;
    }
    if (strcmp(cmd, "CHUNK") == 0) {
        handle_cmd_chunk(args);
        return;
    }
    if (strcmp(cmd, "END") == 0) {
        handle_cmd_end();
        return;
    }

    DEBUG_SERIAL.println("ERR unknown_cmd");
}

void setup() {
    Serial.setRx(PB7_ALT0);
    Serial.setTx(PB6_ALT1);
    Serial.begin(115200);
    delay(300);

    DEBUG_SERIAL.println("=== Tesla PCS UDS Programmer ===");
    DEBUG_SERIAL.println("Serial commands: HELP for protocol");

    pinMode(PIN_PCS_ENABLE, OUTPUT);
    digitalWrite(PIN_PCS_ENABLE, HIGH);
    delay(300);

    ipcCan.begin(500000);
    reset_program_state();

    DEBUG_SERIAL.println("OK READY");
}

void loop() {
    while (DEBUG_SERIAL.available()) {
        char c = (char)DEBUG_SERIAL.read();
        if (c == '\r') continue;

        if (c == '\n') {
            line_buf[line_len] = '\0';
            process_line(line_buf);
            line_len = 0;
            continue;
        }

        if (line_len < (MAX_LINE_LEN - 1)) {
            line_buf[line_len++] = c;
        } else {
            line_len = 0;
            DEBUG_SERIAL.println("ERR line_too_long");
        }
    }

    delay(2);
}
