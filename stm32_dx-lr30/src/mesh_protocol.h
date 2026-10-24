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
    uint32_t cmdSeq;
    uint8_t  cmdType;          // 0 = relay control
    uint8_t  cmdValue;         // bit0 = relay 1 state (0=OFF, 1=ON).
                               // Reserved for future multi-channel support.
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

inline uint16_t meshDeviceId16() {
    volatile const uint32_t* uid0 = (uint32_t*)0x1FFFF7E8;
    volatile const uint32_t* uid1 = (uint32_t*)0x1FFFF7EC;
    volatile const uint32_t* uid2 = (uint32_t*)0x1FFFF7F0;
    uint32_t mix = (*uid0) ^ (*uid1) ^ (*uid2);
    uint16_t id = (uint16_t)(mix ^ (mix >> 16));
    return id ? id : 1;
}