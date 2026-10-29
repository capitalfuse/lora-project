// ============================================================================
//  Packet definitions and shared functions used by mesh_main.cpp
//  (All roles—Node/Repeater/Gateway/Actuator—have been integrated into a
//   single firmware image, so mesh_main.cpp is currently the only file
//   that includes this header.)
//
//  The ESP32 side (esp32_gateway/src/main.cpp) is a separate project with
//  a different compiler and build system, so this header is not included
//  there. The corresponding structures are manually duplicated in the ESP32
//  code. Whenever a field is changed, make sure to update
//  esp32_gateway/src/main.cpp accordingly.
// ============================================================================

#pragma once

#include <Arduino.h>
#include <stddef.h>
#include "siphash24.h"

// ─────────────────────────────────────────────────────────────────────────
//  Packet structures
// ─────────────────────────────────────────────────────────────────────────

// Uplink data transmitted by sensor Nodes (heartbeat/motion events)
struct __attribute__((packed)) MeshPacket {
    uint8_t  magic0, magic1;   // 0xB1, 0x60
    uint16_t originId;
    uint32_t seq;
    uint8_t  hopLimit;
    uint8_t  eventType;        // 0=heartbeat, 1=motion
    uint8_t  payloadLen;
    uint8_t  payload[56];
    uint8_t  crc8;
};

// Acknowledgment for a MeshPacket
struct __attribute__((packed)) AckPacket {
    uint8_t  magic0, magic1;   // 0xAC, 0x60
    uint16_t originId;
    uint32_t seq;
    uint8_t  crc8;
};

// Downlink control command from Gateway to a relay Node
// (Assumes a continuously powered Node within direct radio range)
struct __attribute__((packed)) CmdPacket {
    uint8_t  magic0, magic1;   // 0xC0, 0x60
    uint16_t targetNodeId;
    uint32_t cmdSeq;           // One-time nonce issued by the target Actuator (taken from its last
                               // StatusPacket). Also used as the command id: the Actuator reports it
                               // back in StatusPacket::lastCmdSeqApplied.
    uint8_t  cmdType;          // 0 = relay control
    uint8_t  cmdValue;         // bit0 = relay 1 state (0=OFF, 1=ON).
                               // Reserved for future multi-channel support.
    uint8_t  tag[8];           // SipHash-2-4(key, magic0 .. cmdValue), signed by the ESP32
    uint8_t  crc8;
};

// Status notification from relay Node to Gateway
// (Periodic transmission, immediately after command application,
//  and whenever FEEDBACK_PIN changes)
struct __attribute__((packed)) StatusPacket {
    uint8_t  magic0, magic1;   // 0xB2, 0x60
    uint16_t originId;
    uint32_t seq;
    uint8_t  commandedState;   // State commanded by Gateway (GPIO output value; bit0=relay 1)
    uint8_t  feedbackPinState; // Raw HIGH(1)/LOW(0) level of FEEDBACK_PIN; no interpretation applied
    uint32_t lastCmdSeqApplied;
    uint32_t nonce;            // Current one-time nonce: the next CmdPacket must carry it in cmdSeq
    uint8_t  crc8;
};

// ─────────────────────────────────────────────────────────────────────────
//  Shared functions
// ─────────────────────────────────────────────────────────────────────────
inline uint8_t meshCrc8(const uint8_t* data, uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

// ── Command authentication (SipHash-2-4 tag + Actuator-issued one-time nonce) ──
// Tag input: every byte before `tag` (magic0, magic1, targetNodeId, cmdSeq/nonce, cmdType,
// cmdValue = 10 bytes, little-endian). The ESP32 builds the same 10 bytes (esp32_gateway/src/main.cpp).
inline bool meshCmdVerify(const CmdPacket& p, const uint8_t key[16]) {
    uint64_t t = siphash24(key, (const uint8_t*)&p, offsetof(CmdPacket, tag));
    uint8_t diff = 0;                              // constant-time comparison
    for (uint8_t i = 0; i < 8; i++) diff |= (uint8_t)(t >> (8 * i)) ^ p.tag[i];
    return diff == 0;
}

enum MeshCmdVerdict : uint8_t {
    MCV_REJECT = 0,        // tag does not verify: drop silently
    MCV_APPLY,             // authentic and carries the current nonce: apply, then rotate the nonce
    MCV_RESEND_STATUS      // authentic but stale (nonce already used, e.g. a retry): do not apply,
                           // just report the status again so the Gateway learns the current nonce
};
inline MeshCmdVerdict meshCmdJudge(const CmdPacket& p, const uint8_t key[16], uint32_t currentNonce) {
    if (!meshCmdVerify(p, key)) return MCV_REJECT;
    return (p.cmdSeq == currentNonce) ? MCV_APPLY : MCV_RESEND_STATUS;
}

// New nonce. It only has to avoid repeating (an attacker sees every nonce and tag on the air but
// cannot sign), so the inputs are weak noise (timing, RSSI, ADC) hashed with the key.
// 0xFFFFFFFF is reserved for "no nonce / nothing applied yet".
inline uint32_t meshNonceMix(const uint8_t key[16], uint32_t prev, uint32_t ctr,
                             uint32_t us, int32_t rssi, uint32_t adc) {
    uint32_t v[5] = { prev, ctr, us, (uint32_t)rssi, adc };
    uint64_t h = siphash24(key, (const uint8_t*)v, sizeof(v));
    uint32_t n = (uint32_t)(h ^ (h >> 32));
    if (n == prev) n++;
    if (n == 0xFFFFFFFFUL) n = 0;
    return n;
}

inline uint16_t meshDeviceId16() {
    volatile const uint32_t* uid0 = (uint32_t*)0x1FFFF7E8;
    volatile const uint32_t* uid1 = (uint32_t*)0x1FFFF7EC;
    volatile const uint32_t* uid2 = (uint32_t*)0x1FFFF7F0;
    uint32_t mix = (*uid0) ^ (*uid1) ^ (*uid2);
    uint16_t id = (uint16_t)(mix ^ (mix >> 16));
    return id ? id : 1;
}