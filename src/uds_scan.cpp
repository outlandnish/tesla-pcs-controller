/*
 * UDS Scanner for Tesla Model 3 PCS
 *
 * Request:  0x628 (UDS_pcsRequest, message_id 1576)
 * Response: 0x629 (PCS_udsResponse, message_id 1577)
 *
 * Supports ISO-TP multi-frame TX and RX
 */

#include <Arduino.h>
#include <cstring>
#include "hwdefs.h"
#include "serial_config.h"
#include "debug_serial.h"
#include "../lib/can/can.h"

HardwareSerial Serial1(USART1);
CANBus ipcCan(PIN_CANRX0, PIN_CANTX0);

#define UDS_REQUEST_ID  0x628
#define UDS_RESPONSE_ID 0x629

// UDS Service IDs
#define UDS_DIAG_SESSION_CONTROL    0x10
#define UDS_READ_DATA_BY_ID         0x22
#define UDS_SECURITY_ACCESS         0x27
#define UDS_WRITE_DATA_BY_ID        0x2E
#define UDS_ROUTINE_CONTROL         0x31
#define UDS_TESTER_PRESENT          0x3E

#define DIAG_SESSION_PROGRAMMING    0x02

// Multi-frame response buffer
#define MAX_RESPONSE_LEN 256
uint8_t response_buf[MAX_RESPONSE_LEN];
uint16_t response_len = 0;
bool response_valid = false;

void setup() {
    Serial.setRx(PB7_ALT0);
    Serial.setTx(PB6_ALT1);
    Serial.begin(115200);
    delay(500);

    DEBUG_SERIAL.println("\n=== Tesla PCS UDS Scanner v2 ===");
    DEBUG_SERIAL.println("Request: 0x628 | Response: 0x629\n");

    DEBUG_SERIAL.println("[*] Asserting PCS_ENABLE...");
    pinMode(PIN_PCS_ENABLE, OUTPUT);
    digitalWrite(PIN_PCS_ENABLE, HIGH);
    delay(500);
    DEBUG_SERIAL.println("[+] PCS enabled\n");

    DEBUG_SERIAL.println("[*] Initializing CAN bus...");
    ipcCan.begin(500000);
    DEBUG_SERIAL.println("[+] CAN initialized\n");
}

void print_hex(const uint8_t* data, int len) {
    for (int i = 0; i < len; i++) {
        if (data[i] < 0x10) DEBUG_SERIAL.print("0");
        DEBUG_SERIAL.print(data[i], HEX);
        DEBUG_SERIAL.print(" ");
    }
}

// Send single-frame UDS request
void send_uds(const uint8_t* payload, int payload_len) {
    uint8_t data[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
    data[0] = payload_len;
    int n = (payload_len > 7) ? 7 : payload_len;
    memcpy(&data[1], payload, n);

    DEBUG_SERIAL.print("  TX 0x628: ");
    print_hex(data, 8);
    DEBUG_SERIAL.println();

    ipcCan.sendMessage(UDS_REQUEST_ID, data, 8);
}

// Send multi-frame UDS request (ISO-TP First Frame + Consecutive Frames)
void send_uds_multi(const uint8_t* payload, int payload_len) {
    if (payload_len <= 7) {
        send_uds(payload, payload_len);
        return;
    }

    // First Frame: [10 LEN] + 6 data bytes
    uint8_t ff[8];
    ff[0] = 0x10 | ((payload_len >> 8) & 0x0F);
    ff[1] = payload_len & 0xFF;
    int ff_data = (payload_len < 6) ? payload_len : 6;
    memcpy(&ff[2], payload, ff_data);
    for (int i = ff_data + 2; i < 8; i++) ff[i] = 0xAA;

    DEBUG_SERIAL.print("  TX 0x628 [FF]: ");
    print_hex(ff, 8);
    DEBUG_SERIAL.println();
    ipcCan.sendMessage(UDS_REQUEST_ID, ff, 8);

    // Wait for Flow Control from PCS
    unsigned long fc_start = millis();
    bool got_fc = false;
    while ((millis() - fc_start) < 1000) {
        uint32_t can_id;
        uint8_t data[8];
        uint8_t len;
        if (ipcCan.receiveMessage(can_id, data, len)) {
            if (can_id == UDS_RESPONSE_ID && (data[0] >> 4) == 3) {
                DEBUG_SERIAL.print("  RX 0x629 [FC]: ");
                print_hex(data, len);
                DEBUG_SERIAL.println();
                got_fc = true;
                break;
            }
        }
        delay(1);
    }

    if (!got_fc) {
        DEBUG_SERIAL.println("  [NO FC - aborting multi-frame TX]");
        return;
    }

    // Consecutive Frames
    int offset = 6;
    uint8_t seq = 1;
    while (offset < payload_len) {
        uint8_t cf[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
        cf[0] = 0x20 | (seq & 0x0F);
        int remaining = payload_len - offset;
        int cf_data = (remaining < 7) ? remaining : 7;
        memcpy(&cf[1], &payload[offset], cf_data);

        DEBUG_SERIAL.print("  TX 0x628 [CF seq=");
        DEBUG_SERIAL.print(seq);
        DEBUG_SERIAL.print("]: ");
        print_hex(cf, 8);
        DEBUG_SERIAL.println();
        ipcCan.sendMessage(UDS_REQUEST_ID, cf, 8);

        offset += cf_data;
        seq = (seq + 1) & 0x0F;
        delay(5);  // STmin
    }
}

void send_flow_control() {
    uint8_t fc[8] = {0x30, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
    DEBUG_SERIAL.print("  TX 0x628 [FC]: ");
    print_hex(fc, 8);
    DEBUG_SERIAL.println();
    ipcCan.sendMessage(UDS_REQUEST_ID, fc, 8);
}

// Full ISO-TP receive with multi-frame support
bool wait_response(int timeout_ms = 500) {
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
            // Single Frame
            uint8_t sf_len = data[0] & 0x0F;
            DEBUG_SERIAL.print("  RX 0x629: ");
            print_hex(data, len);

            if (sf_len > 7) sf_len = 7;
            memcpy(response_buf, &data[1], sf_len);
            response_len = sf_len;
            response_valid = true;

            if (sf_len >= 3 && data[1] == 0x7F) {
                DEBUG_SERIAL.print(" [NACK svc=0x");
                if (data[2] < 0x10) DEBUG_SERIAL.print("0");
                DEBUG_SERIAL.print(data[2], HEX);
                DEBUG_SERIAL.print(" nrc=0x");
                if (data[3] < 0x10) DEBUG_SERIAL.print("0");
                DEBUG_SERIAL.print(data[3], HEX);
                DEBUG_SERIAL.print("]");
            } else if (sf_len >= 1 && data[1] >= 0x40 && data[1] < 0x80) {
                DEBUG_SERIAL.print(" [OK svc=0x");
                uint8_t svc = data[1] - 0x40;
                if (svc < 0x10) DEBUG_SERIAL.print("0");
                DEBUG_SERIAL.print(svc, HEX);
                DEBUG_SERIAL.print("]");
            }
            DEBUG_SERIAL.println();
            return true;

        } else if (pci_type == 1) {
            // First Frame
            uint16_t total_len = ((uint16_t)(data[0] & 0x0F) << 8) | data[1];
            if (total_len > MAX_RESPONSE_LEN) total_len = MAX_RESPONSE_LEN;

            DEBUG_SERIAL.print("  RX 0x629 [FF len=");
            DEBUG_SERIAL.print(total_len);
            DEBUG_SERIAL.print("]: ");
            print_hex(data, len);
            DEBUG_SERIAL.println();

            uint16_t copy_len = (total_len < 6) ? total_len : 6;
            memcpy(response_buf, &data[2], copy_len);
            response_len = copy_len;

            send_flow_control();

            // Collect Consecutive Frames
            unsigned long cf_start = millis();
            while (response_len < total_len && (millis() - cf_start) < 2000) {
                if (!ipcCan.receiveMessage(can_id, data, len)) { delay(1); continue; }
                if (can_id != UDS_RESPONSE_ID) continue;
                if ((data[0] >> 4) != 2) continue;

                DEBUG_SERIAL.print("  RX 0x629 [CF seq=");
                DEBUG_SERIAL.print(data[0] & 0x0F);
                DEBUG_SERIAL.print("]: ");
                print_hex(data, len);
                DEBUG_SERIAL.println();

                uint16_t remaining = total_len - response_len;
                uint16_t cf_copy = (remaining < 7) ? remaining : 7;
                memcpy(&response_buf[response_len], &data[1], cf_copy);
                response_len += cf_copy;
            }

            response_valid = true;

            DEBUG_SERIAL.print("  Full response (");
            DEBUG_SERIAL.print(response_len);
            DEBUG_SERIAL.print(" bytes): ");
            print_hex(response_buf, response_len);
            DEBUG_SERIAL.println();
            return true;
        }
    }

    DEBUG_SERIAL.println("  [TIMEOUT]");
    return false;
}

bool is_positive() {
    // 0x7F = negative response indicator, must be excluded!
    return response_valid && response_len >= 1 &&
           response_buf[0] >= 0x40 && response_buf[0] < 0x80 &&
           response_buf[0] != 0x7F;
}

uint8_t get_nrc() {
    if (!response_valid || response_len < 3 || response_buf[0] != 0x7F) return 0;
    return response_buf[2];
}

void loop() {
    static bool scan_done = false;

    if (!scan_done) {
        scan_done = true;

        // ===== STEP 1: Enter Programming Session =====
        DEBUG_SERIAL.println("===== STEP 1: Enter Programming Session =====\n");
        uint8_t prog[] = {UDS_DIAG_SESSION_CONTROL, DIAG_SESSION_PROGRAMMING};
        send_uds(prog, 2);
        wait_response();
        if (!is_positive()) {
            DEBUG_SERIAL.println("\n[!] Failed to enter programming session.");
            return;
        }
        DEBUG_SERIAL.println();
        delay(100);

        // ===== STEP 2: SecurityAccess Unlock =====
        DEBUG_SERIAL.println("===== STEP 2: SecurityAccess Unlock =====\n");
        bool security_unlocked = false;
        {
            // Request seed (sub-function 0x01)
            uint8_t seed_req[] = {UDS_SECURITY_ACCESS, 0x01};
            send_uds(seed_req, 2);
            wait_response(2000);

            if (is_positive() && response_len >= 3) {
                // response_buf: [0x67] [0x01 sub-func] [seed bytes...]
                uint8_t seed_len = response_len - 2;  // minus service byte + sub-function
                if (seed_len > 16) seed_len = 16;
                uint8_t seed[16];
                memcpy(seed, &response_buf[2], seed_len);

                DEBUG_SERIAL.print("  Seed (");
                DEBUG_SERIAL.print(seed_len);
                DEBUG_SERIAL.print(" bytes): ");
                print_hex(seed, seed_len);
                DEBUG_SERIAL.println("\n");

                // Deterministic key algorithm from Tesla docs: key[i] = seed[i] ^ 0x35
                uint8_t key[16];
                for (int b = 0; b < seed_len; b++) {
                    key[b] = seed[b] ^ 0x35;
                }

                DEBUG_SERIAL.print("  Key (seed ^ 0x35): ");
                print_hex(key, seed_len);
                DEBUG_SERIAL.println();

                // Send key (sub-function 0x02) — needs multi-frame for 16-byte key
                uint8_t key_msg[18];  // 0x27 + 0x02 + up to 16 key bytes
                key_msg[0] = UDS_SECURITY_ACCESS;
                key_msg[1] = 0x02;
                memcpy(&key_msg[2], key, seed_len);
                send_uds_multi(key_msg, 2 + seed_len);
                wait_response(2000);

                if (is_positive()) {
                    security_unlocked = true;
                    DEBUG_SERIAL.println("  >>> SECURITY UNLOCKED! <<<\n");
                } else {
                    uint8_t nrc = get_nrc();
                    DEBUG_SERIAL.print("  Rejected (NRC=0x");
                    if (nrc < 0x10) DEBUG_SERIAL.print("0");
                    DEBUG_SERIAL.print(nrc, HEX);
                    DEBUG_SERIAL.println(")\n");
                }
            } else {
                DEBUG_SERIAL.println("  [!] Could not get seed\n");
            }
        }
        delay(200);

        // ===== STEP 3: ODJ DID Read Test =====
        DEBUG_SERIAL.println("===== STEP 3: ODJ DID Read Test =====\n");

        enum DidReadStatus : uint8_t {
            DID_OK = 0,
            DID_NACK = 1,
            DID_TIMEOUT = 2,
            DID_SKIPPED = 3,
        };

        struct DidSummaryEntry {
            uint16_t did;
            const char* label;
            DidReadStatus status;
            uint16_t payload_len;
            uint8_t nrc;
            uint16_t value_len;
            uint8_t value[64];
        };

        DidSummaryEntry did_summary[32];
        uint8_t did_summary_count = 0;

        auto add_summary = [&](uint16_t did,
                               const char* label,
                               DidReadStatus status,
                               uint16_t payload_len,
                               uint8_t nrc,
                               const uint8_t* value,
                               uint16_t value_len) {
            if (did_summary_count >= (sizeof(did_summary) / sizeof(did_summary[0]))) {
                return;
            }

            DidSummaryEntry& e = did_summary[did_summary_count++];
            e.did = did;
            e.label = label;
            e.status = status;
            e.payload_len = payload_len;
            e.nrc = nrc;
            e.value_len = (value_len > sizeof(e.value)) ? sizeof(e.value) : value_len;
            if (e.value_len > 0 && value != nullptr) {
                memcpy(e.value, value, e.value_len);
            }
        };

        auto read_did = [&](uint16_t did, const char* label, int timeout_ms = 1500) {
            uint8_t req[] = {
                UDS_READ_DATA_BY_ID,
                (uint8_t)((did >> 8) & 0xFF),
                (uint8_t)(did & 0xFF)
            };

            DEBUG_SERIAL.print("  Read DID 0x");
            if (did < 0x1000) DEBUG_SERIAL.print("0");
            DEBUG_SERIAL.print(did, HEX);
            DEBUG_SERIAL.print(" (");
            DEBUG_SERIAL.print(label);
            DEBUG_SERIAL.println(")");

            send_uds(req, sizeof(req));
            bool got_response = wait_response(timeout_ms);

            if (got_response && is_positive()) {
                // Positive ReadDID response format: [0x62 DID_H DID_L DATA...]
                uint16_t did_value_len = 0;
                const uint8_t* did_value_ptr = nullptr;
                if (response_len >= 3 && response_buf[0] == 0x62) {
                    did_value_ptr = &response_buf[3];
                    did_value_len = response_len - 3;
                }

                DEBUG_SERIAL.print("  [OK] DID 0x");
                if (did < 0x1000) DEBUG_SERIAL.print("0");
                DEBUG_SERIAL.print(did, HEX);
                DEBUG_SERIAL.print(" len=");
                DEBUG_SERIAL.print(response_len);
                DEBUG_SERIAL.println();
                add_summary(did, label, DID_OK, response_len, 0, did_value_ptr, did_value_len);
            } else if (!got_response) {
                DEBUG_SERIAL.print("  [TIMEOUT] DID 0x");
                if (did < 0x1000) DEBUG_SERIAL.print("0");
                DEBUG_SERIAL.println(did, HEX);
                add_summary(did, label, DID_TIMEOUT, 0, 0, nullptr, 0);
            } else {
                uint8_t nrc = get_nrc();
                DEBUG_SERIAL.print("  [NACK] DID 0x");
                if (did < 0x1000) DEBUG_SERIAL.print("0");
                DEBUG_SERIAL.print(did, HEX);
                DEBUG_SERIAL.print(" NRC=0x");
                if (nrc < 0x10) DEBUG_SERIAL.print("0");
                DEBUG_SERIAL.println(nrc, HEX);
                add_summary(did, label, DID_NACK, response_len, nrc, nullptr, 0);
            }

            delay(60);
        };

        if (security_unlocked) {
            uint8_t tester_present[] = {UDS_TESTER_PRESENT, 0x00};
            send_uds(tester_present, 2);
            wait_response(500);
        }

        // ===== STEP 3A: ODJ Download-Prep Actions =====
        DEBUG_SERIAL.println("\n===== STEP 3A: ODJ Download-Prep Actions =====");

        // Legacy candidate DID for MODULE_TO_PROGRAM write via UDS 0x2E.
        // If this does not respond, update this DID from a known-good legacy trace.
        const uint16_t module_to_program_did = 0xF1A0;

        // Set these to ODJ routine IDs when known. 0xFFFF keeps a step disabled.
        const uint16_t odj_routine_init_module_for_download = 0xFFFF;
        const uint16_t odj_routine_erase_module = 0xFFFF;

        auto write_did_raw = [&](uint16_t did, const uint8_t* data, uint8_t data_len, const char* label) -> bool {
            uint8_t req[32];
            if (data_len > (sizeof(req) - 3)) {
                return false;
            }
            req[0] = UDS_WRITE_DATA_BY_ID;
            req[1] = (uint8_t)((did >> 8) & 0xFF);
            req[2] = (uint8_t)(did & 0xFF);
            memcpy(&req[3], data, data_len);

            DEBUG_SERIAL.print("  WDBI 0x");
            if (did < 0x1000) DEBUG_SERIAL.print("0");
            DEBUG_SERIAL.print(did, HEX);
            DEBUG_SERIAL.print(" data=");
            for (uint8_t i = 0; i < data_len; i++) {
                if (data[i] < 0x10) DEBUG_SERIAL.print("0");
                DEBUG_SERIAL.print(data[i], HEX);
                DEBUG_SERIAL.print(" ");
            }
            DEBUG_SERIAL.print(" (");
            DEBUG_SERIAL.print(label);
            DEBUG_SERIAL.println(")");

            send_uds_multi(req, 3 + data_len);
            bool got = wait_response(1200);
            return got && is_positive();
        };

        auto run_routine_start_u8 = [&](uint16_t routine_id, uint8_t param, const char* label) -> bool {
            uint8_t req[] = {
                UDS_ROUTINE_CONTROL,
                0x01,
                (uint8_t)((routine_id >> 8) & 0xFF),
                (uint8_t)(routine_id & 0xFF),
                param
            };

            DEBUG_SERIAL.print("  RC start 0x");
            if (routine_id < 0x1000) DEBUG_SERIAL.print("0");
            DEBUG_SERIAL.print(routine_id, HEX);
            DEBUG_SERIAL.print(" param=0x");
            if (param < 0x10) DEBUG_SERIAL.print("0");
            DEBUG_SERIAL.print(param, HEX);
            DEBUG_SERIAL.print(" (");
            DEBUG_SERIAL.print(label);
            DEBUG_SERIAL.println(")");

            send_uds(req, sizeof(req));
            bool got = wait_response(1600);
            return got && is_positive();
        };

        if (!security_unlocked) {
            DEBUG_SERIAL.println("  [!] Skipping prep actions (security not unlocked)");
        } else {
            const uint8_t module_candidates[] = {0x01};  // user-selected CPU1 assumption
            const uint8_t key_ascii[] = {'d', 'j', 'n', '1', '4', '4'};

            for (size_t m = 0; m < (sizeof(module_candidates) / sizeof(module_candidates[0])); m++) {
                DEBUG_SERIAL.print("\n  -- Module candidate 0x");
                if (module_candidates[m] < 0x10) DEBUG_SERIAL.print("0");
                DEBUG_SERIAL.print(module_candidates[m], HEX);
                DEBUG_SERIAL.println(" --");

                if (module_to_program_did != 0xFFFF) {
                    uint8_t p1[] = {module_candidates[m]};
                    uint8_t p2[] = {module_candidates[m], 'd', 'j', 'n', '1', '4', '4'};
                    uint8_t p3[] = {'d', 'j', 'n', '1', '4', '4', module_candidates[m]};

                    bool ok1 = write_did_raw(module_to_program_did, p1, sizeof(p1), "MODULE_TO_PROGRAM value-only");
                    DEBUG_SERIAL.println(ok1 ? "    [OK] value-only accepted" : "    [NACK] value-only rejected");

                    bool ok2 = write_did_raw(module_to_program_did, p2, sizeof(p2), "MODULE_TO_PROGRAM value+key(djn144)");
                    DEBUG_SERIAL.println(ok2 ? "    [OK] value+key accepted" : "    [NACK] value+key rejected");

                    bool ok3 = write_did_raw(module_to_program_did, p3, sizeof(p3), "MODULE_TO_PROGRAM key(djn144)+value");
                    DEBUG_SERIAL.println(ok3 ? "    [OK] key+value accepted" : "    [NACK] key+value rejected");

                    bool any_ok = ok1 || ok2 || ok3;
                    if (!any_ok) {
                        DEBUG_SERIAL.println("    [!] MODULE_TO_PROGRAM write failed for all payload formats");
                    }
                } else {
                    DEBUG_SERIAL.println("    [SKIP] MODULE_TO_PROGRAM DID not configured");
                }

                if (odj_routine_init_module_for_download != 0xFFFF) {
                    bool ok = run_routine_start_u8(odj_routine_init_module_for_download, module_candidates[m], "INIT_MODULE_FOR_DOWNLOAD");
                    DEBUG_SERIAL.println(ok ? "    [OK] INIT_MODULE_FOR_DOWNLOAD accepted" : "    [NACK] INIT_MODULE_FOR_DOWNLOAD failed");
                } else {
                    DEBUG_SERIAL.println("    [SKIP] INIT_MODULE_FOR_DOWNLOAD routine ID not configured");
                }

                if (odj_routine_erase_module != 0xFFFF) {
                    bool ok = run_routine_start_u8(odj_routine_erase_module, module_candidates[m], "ERASE_MODULE");
                    DEBUG_SERIAL.println(ok ? "    [OK] ERASE_MODULE accepted" : "    [NACK] ERASE_MODULE failed");
                } else {
                    DEBUG_SERIAL.println("    [SKIP] ERASE_MODULE routine ID not configured");
                }
            }
        }

        // Security level 0 DIDs from ODJ
        read_did(0x0101, "COMP_AND_FW_TYPE");
        read_did(0x0301, "SENSORS_DATA");
        read_did(0xF000, "Application_CRC");
        read_did(0xF002, "Subcomponent2_CRC");
        read_did(0xF005, "COMPONENT_BUILD_TYPE");
        read_did(0xF006, "BUILD_CONFIG");
        read_did(0xF007, "BOOTLOADER_CRC");
        read_did(0xF012, "BOARD_PART_NUMBER");
        read_did(0xF013, "BOARD_SERIAL_NUMBER");
        read_did(0xF014, "PACKAGE_PART_NUMBER");
        read_did(0xF015, "PACKAGE_SERIAL_NUMBER");
        read_did(0xF01C, "ASSEMBLY_ID");
        read_did(0xF01D, "USAGE_ID");
        read_did(0xF01E, "SUB_USAGE_ID");
        read_did(0xF030, "SUB_PACKAGE_PART_NUMBER");
        read_did(0xF031, "SUB_PACKAGE_SERIAL_NUMBER");
        read_did(0xF180, "BOOTLOADER_VERSION");

        // Optional security level 5 DID from ODJ
        DEBUG_SERIAL.println("\n  Attempting security level 5 for protected DIDs...");
        bool level5_unlocked = false;
        {
            uint8_t seed_req[] = {UDS_SECURITY_ACCESS, 0x05};
            send_uds(seed_req, 2);
            wait_response(2000);

            if (is_positive() && response_len >= 3 && response_buf[1] == 0x05) {
                uint8_t seed_len = response_len - 2;
                if (seed_len > 16) seed_len = 16;
                uint8_t key_msg[18];
                key_msg[0] = UDS_SECURITY_ACCESS;
                key_msg[1] = 0x06;
                for (int b = 0; b < seed_len; b++) {
                    key_msg[2 + b] = response_buf[2 + b] ^ 0x35;
                }

                send_uds_multi(key_msg, 2 + seed_len);
                wait_response(2000);
                level5_unlocked = is_positive();
            }
        }

        if (level5_unlocked) {
            DEBUG_SERIAL.println("  [OK] Security level 5 unlocked");
            read_did(0xEE00, "Read_EEPROM_Validity_Region", 2500);
        } else {
            DEBUG_SERIAL.println("  [!] Security level 5 unlock failed; skipping DID 0xEE00");
            add_summary(0xEE00, "Read_EEPROM_Validity_Region", DID_SKIPPED, 0, 0, nullptr, 0);
        }

        DEBUG_SERIAL.println("\n===== ODJ DID Summary =====");
        for (uint8_t i = 0; i < did_summary_count; i++) {
            DEBUG_SERIAL.print("  0x");
            if (did_summary[i].did < 0x1000) DEBUG_SERIAL.print("0");
            DEBUG_SERIAL.print(did_summary[i].did, HEX);
            DEBUG_SERIAL.print(" -> ");

            switch (did_summary[i].status) {
                case DID_OK:
                    DEBUG_SERIAL.print("OK len=");
                    DEBUG_SERIAL.print(did_summary[i].payload_len);
                    break;
                case DID_NACK:
                    DEBUG_SERIAL.print("NACK nrc=0x");
                    if (did_summary[i].nrc < 0x10) DEBUG_SERIAL.print("0");
                    DEBUG_SERIAL.print(did_summary[i].nrc, HEX);
                    break;
                case DID_TIMEOUT:
                    DEBUG_SERIAL.print("TIMEOUT");
                    break;
                case DID_SKIPPED:
                    DEBUG_SERIAL.print("SKIPPED");
                    break;
            }

            DEBUG_SERIAL.print(" : ");
            DEBUG_SERIAL.println(did_summary[i].label);

            if (did_summary[i].status == DID_OK && did_summary[i].value_len > 0) {
                DEBUG_SERIAL.print("      value=");
                for (uint16_t b = 0; b < did_summary[i].value_len; b++) {
                    if (did_summary[i].value[b] < 0x10) DEBUG_SERIAL.print("0");
                    DEBUG_SERIAL.print(did_summary[i].value[b], HEX);
                    DEBUG_SERIAL.print(" ");
                }
                DEBUG_SERIAL.println();
            }
        }

        // ===== DONE =====
        DEBUG_SERIAL.println("\n[+] Scan complete");
        DEBUG_SERIAL.println("Enter 'R' to rescan");
    }

    if (DEBUG_SERIAL.available()) {
        char cmd = DEBUG_SERIAL.read();
        if (cmd == 'R' || cmd == 'r') {
            scan_done = false;
        }
    }

    delay(100);
}
