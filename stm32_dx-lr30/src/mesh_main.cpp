// ============================================================================
//  Firmware dedicated to the bluepill_mesh environment
//  Shared firmware for Node / Repeater / Gateway / Actuator
//  (A single build runs on the same STM32F103C8 Blue Pill board for all devices.
//   The role is determined only by wiring, jumpers, and ESP32 presence.)
//
//  Role determination logic:
//    1) ESP32 responds to the UART handshake          -> ROLE_GATEWAY
//    2) ACTUATOR_JUMPER (PB1) is connected to GND     -> ROLE_ACTUATOR
//    3) ROLE_JUMPER (PB0) is connected to GND         -> ROLE_REPEATER
//    4) Both are open                                -> ROLE_NODE (low-power sensor)
//    If both PB0 and PB1 are grounded, treat this as a configuration error
//      and fall back to the safer ROLE_NODE role.
//
//  Pin assignments (same STM32F103C8 Blue Pill board with DX-LR30 for all roles):
//    SPI1  (DX-LR30)      : PA4=NSS, PA5=SCK, PA6=MISO, PA7=MOSI
//    USART1 (debug output) : PA9=TX, PA10=RX   Used only when MESH_DEBUG_SERIAL=1
//    USART3(ESP32/Gateway) : PB10=TX, PB11=RX
//    I2C1  (BMI160)        : PB8=SCL, PB9=SDA        Used only by ROLE_NODE
//    BMI160 INT1            : PB2 (wake from STOP mode via Any-Motion interrupt)
//    Role jumper (PB0)      : GND = Repeater; open = continue checking Node/Gateway/Actuator
//    Role jumper (PB1)      : GND = Actuator; open = continue checking Node/Gateway/Repeater
//      (PB0/PB1 use only the internal pull-ups. No external resistors or connection to VCC are needed.
//       The two states (grounded/open) on two pins distinguish the four roles.)
//    RELAY_PIN               : PB3  Used only by ROLE_ACTUATOR
//    FEEDBACK_PIN            : PB4  Used only by ROLE_ACTUATOR (transmits raw HIGH/LOW)

//
//  [Requires on-site adjustment]
//    - BMI160 I2C address (0x68/0x69; verify by scanning I2C on the actual hardware)
//    - Any-Motion interrupt register values (threshold, duration, and bit layout must be
//      verified against the register map in the Bosch datasheet before writing)
//    - ACK_TIMEOUT_MS (adjust after measuring ToA in the bluepill_cli environment)
//    - RF settings (freq/sf/bw/cr) must be identical for all roles
//    - Actual wiring and polarity of RELAY_PIN/FEEDBACK_PIN (Actuator role only)
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <STM32RTC.h>
#include <STM32LowPower.h>
#include "sx1262_driver.h"
#include "mesh_protocol.h"

// ─────────────────────────────────────────────────────────────────────────
//  Pin and constant definitions
// ─────────────────────────────────────────────────────────────────────────
#define ROLE_PIN        PB0    // GND = Repeater; open = Node/Gateway/Actuator
#define ACTUATOR_PIN    PB1    // GND = Actuator; open = Node/Gateway/Repeater
                                // PB0/PB1 are reserved exclusively for role detection; do not use them for other purposes.
#define BMI160_INT_PIN  PB3    // Wake from STOP mode via Any-Motion interrupt
#define RELAY_PIN       PB4    // Used only by ROLE_ACTUATOR
#define FEEDBACK_PIN    PB5    // Used only by ROLE_ACTUATOR

// [Known issue, workaround in place] Explicitly calling rtc.setClockSource(LSE_CLOCK)
// conflicts with the STM32LowPower RTC alarm configuration and can prevent the device
// from waking up from deepSleep. This issue was reported as stm32duino/STM32LowPower issue #22.
// The LSE crystal has been verified to oscillate correctly on the actual hardware (by direct register inspection),
// but to avoid this known bug, do not explicitly call setClockSource().
// Let LowPower.begin() use its default clock configuration.
// This limitation has no practical impact because the design does not need to maintain absolute time.

#ifndef MESH_DEBUG_SERIAL
#define MESH_DEBUG_SERIAL 1
#endif

#if MESH_DEBUG_SERIAL
#define DBG(x)      Serial1.print(x)
#define DBGLN(x)    Serial1.println(x)
#else
#define DBG(x)
#define DBGLN(x)
#endif

static const uint32_t RF_FREQUENCY_HZ   = 920000000UL; // Verify applicable regional regulations (e.g. ARIB STD-T108)
static const uint8_t  RF_SF             = 9;
static const float    RF_BW_KHZ         = 125.0f;
static const uint8_t  RF_CR             = 5;
static const int8_t   RF_TX_POWER_DBM   = 13;

static const uint8_t  MAX_RETRIES       = 3;
static const uint32_t ACK_TIMEOUT_MS    = 150;   // [Requires measurement] Revisit after measuring ToA with bluepill_cli
static const uint32_t RETRY_BACKOFF_MIN = 100;
static const uint32_t RETRY_BACKOFF_MAX = 400;
static const uint32_t RELAY_JITTER_MIN  = 20;
static const uint32_t RELAY_JITTER_MAX  = 150;

static const uint32_t HEARTBEAT_INTERVAL_SEC = 3600;    // [For testing] Normally 3600 (1 hour); restore after testing
static const uint32_t MOTION_COOLDOWN_MS     = 5000;  // Suppress retransmissions during continuous motion

static const uint8_t  CMD_MAX_RETRIES        = 3;     // Number of retries for Gateway-to-relay-Node commands
static const uint32_t CMD_CONFIRM_TIMEOUT_MS = 400;   // Wait for StatusPacket confirmation from relay Node (can be short because RX is always on)
static const uint32_t CMD_RETRY_BACKOFF_MIN  = 100;
static const uint32_t CMD_RETRY_BACKOFF_MAX  = 300;

static const uint8_t  BMI160_ADDR       = 0x69; // [Verify] Scan I2C on the actual hardware
static const uint8_t  BMI160_CHIP_ID    = 0xD1; // Expected WHO_AM_I value

static const uint8_t  DEFAULT_HOP_LIMIT = 3;
static const uint8_t  SEEN_CACHE_SIZE   = 24;

static const uint32_t ACTUATOR_STATUS_INTERVAL_MS   = 60000; // Periodic status transmission (also serves as a heartbeat)
static const uint32_t ACTUATOR_FEEDBACK_DEBOUNCE_MS = 200;   // Debounce the feedback signal

// ─────────────────────────────────────────────────────────────────────────
//  Roles
// ─────────────────────────────────────────────────────────────────────────
enum DeviceRole : uint8_t { ROLE_NODE, ROLE_REPEATER, ROLE_GATEWAY, ROLE_ACTUATOR };
static DeviceRole myRole;
static uint16_t   myNodeId;
static uint32_t   mySeq = 0;
static bool       sensorAvailable = false;

// Actuator-only state (unused by other roles; consumes only a few extra bytes of RAM)
static uint8_t  actCommandedState = 0;               // bit0 = commanded state for relay 1
static uint8_t  actFeedbackPinState = 0;              // Raw HIGH(1)/LOW(0) level of FEEDBACK_PIN
static uint32_t actLastCmdSeqApplied = 0xFFFFFFFFUL;  // Initial value indicating that no command has been applied
static uint32_t actLastStatusSentMs = 0;
static uint32_t actStatusSeq = 0;
static uint8_t  actDebounceRawState = 0;
static uint32_t actDebounceSinceMs = 0;

static SX1262Radio radio;
// USART2 (ESP32 communication): In this STM32duino core’s Blue Pill variant,
// the framework has already declared `Serial3` (Uart type, default pins PB10=TX/PB11=RX)
// as a global object. Since these match the intended pins,
// use this Serial3 instance without declaring another HardwareSerial object.
// (Note that HardwareSerial is an abstract base class and cannot be instantiated directly.)

static STM32RTC& rtc = STM32RTC::getInstance();

// Packet structures, crc8(), and deviceId16() have been moved to mesh_protocol.h.
// Same-name wrappers for crc8()/deviceId16() are provided for compatibility with existing callers.
static inline uint8_t crc8(const uint8_t* data, uint16_t len) { return meshCrc8(data, len); }
static inline uint16_t deviceId16() { return meshDeviceId16(); }

// ─────────────────────────────────────────────────────────────────────────
//  Duplicate-detection cache (prevents relay loops)
// ─────────────────────────────────────────────────────────────────────────
struct SeenEntry { uint16_t originId; uint32_t seq; bool used; };
static SeenEntry seenCache[SEEN_CACHE_SIZE];
static uint8_t   seenIdx = 0;

static bool alreadySeen(uint16_t originId, uint32_t seq) {
    for (uint8_t i = 0; i < SEEN_CACHE_SIZE; i++)
        if (seenCache[i].used && seenCache[i].originId == originId && seenCache[i].seq == seq)
            return true;
    return false;
}
static void markSeen(uint16_t originId, uint32_t seq) {
    seenCache[seenIdx] = {originId, seq, true};
    seenIdx = (uint8_t)((seenIdx + 1) % SEEN_CACHE_SIZE);
}

// ─────────────────────────────────────────────────────────────────────────
//  LED management (non-blocking)
// ─────────────────────────────────────────────────────────────────────────
//#ifndef ENABLE_STATUS_LED
//#define ENABLE_STATUS_LED 1   // Set to 0 to maximize battery life on Nodes
//#endif

//static uint32_t ledOffAt = 0;

// static void ledPulse(uint16_t ms) {
// #if ENABLE_STATUS_LED
//     digitalWrite(LED_PIN, HIGH);
//     ledOffAt = millis() + ms;
// #endif
// }

// static void ledService() {
// #if ENABLE_STATUS_LED
//     if (ledOffAt != 0 && millis() >= ledOffAt) {
//         digitalWrite(LED_PIN, LOW);
//         ledOffAt = 0;
//     }
// #endif
// }

// static void ledWaitOff() {
//     while (ledOffAt != 0) ledService();
// }

// static void blinkRolePattern(DeviceRole role) {
//     uint8_t count = (role == ROLE_NODE) ? 1 :
//                     (role == ROLE_REPEATER) ? 2 :
//                     (role == ROLE_GATEWAY) ? 3 : 4; // ROLE_ACTUATOR = 4 flashes
//     for (uint8_t i = 0; i < count; i++) {
//         digitalWrite(LED_PIN, HIGH);  delay(150);
//         digitalWrite(LED_PIN, LOW); delay(150);
//     }
//     delay(400);
// }

// static void blinkErrorPattern(uint8_t times) {
//     for (uint8_t i = 0; i < times; i++) {
//         digitalWrite(LED_PIN, HIGH);  delay(60);
//         digitalWrite(LED_PIN, LOW); delay(60);
//     }
// }

// ─────────────────────────────────────────────────────────────────────────
//  BMI160: direct register access (Any-Motion interrupt configuration)
//  The library API does not cover the interrupt engine, so I2C is used directly for configuration only.
// ─────────────────────────────────────────────────────────────────────────
static void bmi160WriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(BMI160_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t bmi160ReadReg(uint8_t reg) {
    Wire.beginTransmission(BMI160_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)BMI160_ADDR, 1);
    return Wire.available() ? Wire.read() : 0xFF;
}

static bool bmi160Present() {
    uint8_t whoami = bmi160ReadReg(0x00); // CHIP_ID register
    return whoami == BMI160_CHIP_ID;
}

static void bmi160SetupAnyMotionInterrupt() {
    // Switch PMU modes (0x7E = CMD register)
    bmi160WriteReg(0x7E, 0x12); delay(5); // ACC_PMU_MODE = Low Power
    bmi160WriteReg(0x7E, 0x14); delay(5); // GYR_PMU_MODE = Suspend (restore after wake-up if needed)

    bmi160WriteReg(0x40, 0x28); // ACC_CONF: low ODR + undersampling [verify against datasheet]
    bmi160WriteReg(0x41, 0x03); // ACC_RANGE: ±2g

    bmi160WriteReg(0x5F, 0x0A); // INT_MOTION_0: Any-Motion threshold [requires testing and adjustment]
    bmi160WriteReg(0x60, 0x01); // INT_MOTION_1: duration [requires testing and adjustment]

    bmi160WriteReg(0x50, 0x07); // INT_EN_0: anymo_x/y/z_en [verify against datasheet]
    bmi160WriteReg(0x55, 0x04); // INT_MAP_0: map Any-Motion to INT1
    bmi160WriteReg(0x53, 0x0A); // INT_OUT_CTRL: INT1 push-pull, active-high
    bmi160WriteReg(0x54, 0x00); // INT_LATCH: non-latched
}

static void bmi160WakeGyro() {
    bmi160WriteReg(0x7E, 0x15); // GYR_PMU_MODE = Normal
    delay(60);                  // Wait for startup stabilization (adjust to the datasheet value)
}

static void bmi160SleepGyro() {
    bmi160WriteReg(0x7E, 0x14); // GYR_PMU_MODE = Suspend
}

static void bmi160ClearInterrupt() {
    bmi160ReadReg(0x1C); // Reading INT_STATUS_0 clears the latched interrupt
}

// Placeholder implementation: read the accelerometer/gyroscope registers (0x12-0x17 accel, 0x0C-0x11 gyro)
// and place scaled values into mp.payload. This can be replaced with reads using a library
// such as DFRobot_BMI160.
/* static uint8_t bmi160ReadMotionPayload(uint8_t* out) {
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    // TODO: Replace with the actual sensor-reading implementation
    memcpy(&out[0], &ax, 2); memcpy(&out[2], &ay, 2); memcpy(&out[4], &az, 2);
    memcpy(&out[6], &gx, 2); memcpy(&out[8], &gy, 2); memcpy(&out[10], &gz, 2);
    return 12;
} */

// Read consecutive registers (burst read)
static bool bmi160ReadRegs(uint8_t startReg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(BMI160_ADDR);
    Wire.write(startReg);
    if (Wire.endTransmission(false) != 0) return false;   // repeated start

    uint8_t got = Wire.requestFrom((int)BMI160_ADDR, (int)len);
    if (got != len) return false;

    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

static uint8_t bmi160ReadMotionPayload(uint8_t* out) {
    // 0x0C..0x11 = GYR X/Y/Z (LSB and MSB for each axis)
    // 0x12..0x17 = ACC X/Y/Z (LSB and MSB for each axis)
    uint8_t raw[12];
    if (!bmi160ReadRegs(0x0C, raw, 12)) {
        DBGLN("[WARN] BMI160 read failed");
        return 0;   // Send with payloadLen=0 to indicate a read failure
    }

    // raw is ordered gyro -> accel. Reorder it to the existing payload order (ax, ay, az, gx, gy, gz).
    memcpy(&out[0],  &raw[6],  6);   // accel X,Y,Z
    memcpy(&out[6],  &raw[0],  6);   // gyro  X,Y,Z
    return 12;
}

// ─────────────────────────────────────────────────────────────────────────
//  Role determination
// ─────────────────────────────────────────────────────────────────────────
static bool checkEsp32Present() {
    for (uint8_t attempt = 0; attempt < 5; attempt++) {
        while (Serial3.available()) Serial3.read();
        Serial3.write((uint8_t)0xF0);

        uint32_t t0 = millis();
        while (millis() - t0 < 300) {
            if (Serial3.available() && Serial3.read() == 0xF1) {
                return true;
            }
        }
    }
    return false;
}

static bool roleConfigError = false; // Warning flag if ROLE_PIN and ACTUATOR_PIN are both grounded

static DeviceRole determineRole() {
    pinMode(ROLE_PIN, INPUT_PULLUP);
    pinMode(ACTUATOR_PIN, INPUT_PULLUP);
    delay(5);
    bool repeaterJumper = (digitalRead(ROLE_PIN) == LOW);
    bool actuatorJumper = (digitalRead(ACTUATOR_PIN) == LOW);

    if (checkEsp32Present()) return ROLE_GATEWAY;

    if (repeaterJumper && actuatorJumper) {
        // Configuration error (both jumpers grounded simultaneously). Fall back to the safer Node role,
        // and indicate the error pattern in setup() so it can be noticed.
        roleConfigError = true;
        return ROLE_NODE;
    }
    if (actuatorJumper) return ROLE_ACTUATOR;
    if (repeaterJumper) return ROLE_REPEATER;
    return ROLE_NODE;
}

// ─────────────────────────────────────────────────────────────────────────
//  RTC sleep (Node only)
// ─────────────────────────────────────────────────────────────────────────
static void enterStopModeUntilNextEvent(uint32_t sleepSeconds) {
//    ledWaitOff();
#if MESH_DEBUG_SERIAL
    DBG("[NODE] entering stop mode for ");
    DBG(sleepSeconds);
    DBGLN("s");
    Serial1.flush();
#endif
#ifdef DISABLE_LOW_POWER_SLEEP
    // Debug builds only: entering STOP mode stops the clock and disconnects the SWD debugger,
    // so use a regular delay() instead.
    // (Power consumption cannot be tested, but code behavior can be checked and execution can be stepped
    //   through breakpoints.)
    delay(sleepSeconds * 1000UL);
#else
    LowPower.deepSleep(sleepSeconds * 1000UL);
#endif
    DBGLN("[NODE] woke up from stop mode");
    // Wake-up source: RTC alarm expiration or BMI160 INT1 (determined by motionWakeFlag)
}

static volatile bool motionWakeFlag = false;
static void onMotionWake() { motionWakeFlag = true; }

// ─────────────────────────────────────────────────────────────────────────
//  UART communication with ESP32 (Gateway only)
//  Uplink: [0xAA][0x55][LEN(2)][RSSI(2)][SNR(2)][LoRaLen][LoRaPayload...][CRC8] -> ESP32
//        Response: 0x01 = publish succeeded / 0x02 = failed (waited for inside forwardToEsp32)
//  Downlink: [0xCC][0x33][targetNodeId(2)][cmdType][cmdValue][CRC8(unused)] <- ESP32
//        (Relay control command received via MQTT subscription)
//  Both directions must be polled continuously over the same Serial3 interface,
//  so all reception is handled in one place (pollEsp32Downlink), and
//  forwardToEsp32 must not read directly (distributed reads can cause data loss).
// ─────────────────────────────────────────────────────────────────────────
static volatile uint8_t lastForwardResult = 0; // 0=pending, 0x01=success, 0x02=failure

enum Esp32RxState {
    E_IDLE, E_CMD_MAGIC2, E_CMD_TARGET_L, E_CMD_TARGET_H, E_CMD_TYPE, E_CMD_VALUE, E_CMD_CRC
};
static Esp32RxState esp32RxState = E_IDLE;
static uint16_t     esp32CmdTarget = 0;
static uint8_t      esp32CmdType = 0;
static uint8_t      esp32CmdValue = 0;
static uint8_t      esp32CmdBuf[4];   // target(2), type, value

static bool sendCommandToActuator(uint16_t targetId, uint8_t cmdType, uint8_t cmdValue);
static void handleIncomingMeshPacket(const PacketInfo& pkt);

struct PendingCmd { uint16_t target; uint8_t type; uint8_t value; };
static const uint8_t CMD_QUEUE_SIZE = 4;
static PendingCmd cmdQueue[CMD_QUEUE_SIZE];
static uint8_t cmdQHead = 0, cmdQTail = 0;

static bool cmdQueuePush(uint16_t t, uint8_t ty, uint8_t v) {
    uint8_t next = (cmdQTail + 1) % CMD_QUEUE_SIZE;
    if (next == cmdQHead) return false;   // Drop the command if the queue is full
    cmdQueue[cmdQTail] = { t, ty, v };
    cmdQTail = next;
    return true;
}
static bool cmdQueuePop(PendingCmd& out) {
    if (cmdQHead == cmdQTail) return false;
    out = cmdQueue[cmdQHead];
    cmdQHead = (cmdQHead + 1) % CMD_QUEUE_SIZE;
    return true;
}

static void pollEsp32Downlink() {
    while (Serial3.available()) {
        uint8_t b = Serial3.read();
        switch (esp32RxState) {
            case E_IDLE:
                if (b == 0x01 || b == 0x02) {
                    lastForwardResult = b;
                } else if (b == 0xCC) {
                    esp32RxState = E_CMD_MAGIC2;
                }
                break;
            case E_CMD_MAGIC2:
                esp32RxState = (b == 0x33) ? E_CMD_TARGET_L : E_IDLE;
                break;
            case E_CMD_TARGET_L:
                esp32CmdTarget = b;
                esp32CmdBuf[0] = b;
                esp32RxState = E_CMD_TARGET_H;
                break;
            case E_CMD_TARGET_H:
                esp32CmdTarget |= ((uint16_t)b << 8);
                esp32CmdBuf[1] = b;
                esp32RxState = E_CMD_TYPE;
                break;
            case E_CMD_TYPE:
                esp32CmdType = b;
                esp32CmdBuf[2] = b;
                esp32RxState = E_CMD_VALUE;
                break;
            case E_CMD_VALUE:
                esp32CmdValue = b;
                esp32CmdBuf[3] = b;
                esp32RxState = E_CMD_CRC;
                break;
            case E_CMD_CRC:
                if (b != crc8(esp32CmdBuf, 4)) {
                    DBGLN("[GW] cmd CRC mismatch, dropped");
                } else {
                    DBG("[GW] cmd queued: target=");
                    DBG(esp32CmdTarget);
                    DBG(" value=");
                    DBGLN(esp32CmdValue);
                    if (!cmdQueuePush(esp32CmdTarget, esp32CmdType, esp32CmdValue)) {
                        DBGLN("[GW] cmd queue full, dropped");
                    }
                }
                esp32RxState = E_IDLE;
                break;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Forward frames to ESP32 (Gateway only)
// ─────────────────────────────────────────────────────────────────────────
static void forwardToEsp32(const PacketInfo& pkt) {
    uint8_t frame[300];
    uint16_t idx = 0;
    frame[idx++] = 0xAA; frame[idx++] = 0x55;

    uint16_t lenPos = idx; idx += 2;

    int16_t rssi = pkt.rssi;
    int16_t snr  = pkt.snr;
    memcpy(&frame[idx], &rssi, 2); idx += 2;
    memcpy(&frame[idx], &snr,  2); idx += 2;
    frame[idx++] = pkt.length;
    memcpy(&frame[idx], pkt.data, pkt.length); idx += pkt.length;

    uint16_t payloadLen = idx - lenPos - 2;
    frame[lenPos]     = payloadLen & 0xFF;
    frame[lenPos + 1] = payloadLen >> 8;

    frame[idx] = crc8(&frame[2], idx - 2);
    idx++;

    Serial3.write(frame, idx);

    // Briefly wait for the MQTT publish result from ESP32 (0x01=success/0x02=failure) and update the status
    lastForwardResult = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 50) {
        pollEsp32Downlink(); // Use the same parser so incoming downlink commands are handled correctly
        if (lastForwardResult != 0) break;
    }
    // if (lastForwardResult == 0x01) {
    //     ledPulse(10);
    // } else if (lastForwardResult == 0x02) {
    //     ledPulse(10); delay(30); ledPulse(10);
    // } else {
    //     ledPulse(10); delay(30); ledPulse(10); delay(30); ledPulse(10); // No response = three flashes
    // }
}

static void dispatchGatewayPacket(const PacketInfo& pkt) {
    if (pkt.crcError || pkt.length < 2) return;
    if (pkt.data[0] == 0xB2 && pkt.data[1] == 0x60) {
        forwardToEsp32(pkt);               // Status from another Actuator
    } else if (pkt.data[0] == 0xB1 && pkt.data[1] == 0x60) {
        handleIncomingMeshPacket(pkt);     // Node heartbeat and other packets
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Send downlink commands to relay Nodes and confirm execution (Gateway only)
//  Relay Nodes are assumed to be continuously powered, always receiving, and within direct radio range,
//  so power-saving measures like those in sendWithAck() for Nodes are unnecessary. The implementation simply
//  sends a command, waits for a StatusPacket confirmation, and retries on timeout.
// ─────────────────────────────────────────────────────────────────────────
static bool sendCommandToActuator(uint16_t targetId, uint8_t cmdType, uint8_t cmdValue) {
    static uint32_t cmdSeqCounter = 0;
    uint32_t seq = cmdSeqCounter++;

    for (uint8_t attempt = 0; attempt < CMD_MAX_RETRIES; attempt++) {
        CmdPacket cp{};
        cp.magic0 = 0xC0; cp.magic1 = 0x60;
        cp.targetNodeId = targetId;
        cp.cmdSeq = seq;
        cp.cmdType = cmdType;
        cp.cmdValue = cmdValue;
        cp.crc8 = crc8((uint8_t*)&cp, sizeof(cp) - 1);

        radio.send((const uint8_t*)&cp, sizeof(cp));
        radio.startReceive();
        // ledPulse(10);

        uint32_t t0 = millis();
        bool confirmed = false;
        PacketInfo pkt;
        while (millis() - t0 < CMD_CONFIRM_TIMEOUT_MS) {
            if (!radio.checkForPacket(pkt)) continue;
            if (pkt.crcError) continue;

            if (pkt.length == sizeof(StatusPacket) &&
                pkt.data[0] == 0xB2 && pkt.data[1] == 0x60) {
                StatusPacket sp;
                memcpy(&sp, pkt.data, sizeof(sp));
                if (sp.originId == targetId && sp.lastCmdSeqApplied == seq) {
                    confirmed = true;
                    break;
                }
            }
            dispatchGatewayPacket(pkt);   // Process unrelated packets instead of discarding them
        }

        if (confirmed) {
            DBGLN("[GW] command confirmed by actuator");
            forwardToEsp32(pkt);
            return true;
        }
        if (attempt < CMD_MAX_RETRIES - 1) {
            delay(random(CMD_RETRY_BACKOFF_MIN, CMD_RETRY_BACKOFF_MAX));
        }
    }
    DBGLN("[GW] command NOT confirmed after retries");
    return false;
}

// ─────────────────────────────────────────────────────────────────────────
//  Send with ACK (Node only)
// ─────────────────────────────────────────────────────────────────────────
static bool waitForAck(uint16_t originId, uint32_t seq, uint32_t timeoutMs) {
    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        PacketInfo pkt;
        if (radio.checkForPacket(pkt) && !pkt.crcError && pkt.length == sizeof(AckPacket)) {
            AckPacket ack;
            memcpy(&ack, pkt.data, sizeof(ack));
            if (ack.magic0 == 0xAC && ack.magic1 == 0x60 &&
                ack.originId == originId && ack.seq == seq) {
                return true;
            }
        }
    }
    return false;
}

static bool sendWithAck(MeshPacket& mp) {
    for (uint8_t attempt = 0; attempt < MAX_RETRIES; attempt++) {
        mp.crc8 = crc8((uint8_t*)&mp, sizeof(mp) - 1);
        radio.send((const uint8_t*)&mp, sizeof(mp));
        // ledPulse(10);

        radio.startReceive();
        bool ok = waitForAck(mp.originId, mp.seq, ACK_TIMEOUT_MS);
        radio.goStandby();

        if (ok) {
            // ledPulse(5);
            return true;
        }
        if (attempt < MAX_RETRIES - 1) {
            delay(random(RETRY_BACKOFF_MIN, RETRY_BACKOFF_MAX));
        }
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────
//  Incoming packet processing (shared by Repeater/Gateway)
// ─────────────────────────────────────────────────────────────────────────
static uint8_t buildAck(const MeshPacket& mp, uint8_t* out) {
    AckPacket ack{};
    ack.magic0 = 0xAC; ack.magic1 = 0x60;
    ack.originId = mp.originId;
    ack.seq = mp.seq;
    ack.crc8 = crc8((uint8_t*)&ack, sizeof(ack) - 1);
    memcpy(out, &ack, sizeof(ack));
    return sizeof(ack);
}

static void relayPacket(MeshPacket& mp) {
    if (mp.hopLimit == 0) return;
    mp.hopLimit--;
    delay(random(RELAY_JITTER_MIN, RELAY_JITTER_MAX));
    mp.crc8 = crc8((uint8_t*)&mp, sizeof(mp) - 1);
    radio.send((const uint8_t*)&mp, sizeof(mp));
    radio.startReceive();
    // ledPulse(15);
}

static void handleIncomingMeshPacket(const PacketInfo& pkt) {
    if (pkt.crcError || pkt.length < sizeof(MeshPacket)) return;

    MeshPacket mp;
    memcpy(&mp, pkt.data, sizeof(mp));
    if (mp.magic0 != 0xB1 || mp.magic1 != 0x60) return;
    if (alreadySeen(mp.originId, mp.seq)) return;
    markSeen(mp.originId, mp.seq);

    // 1. Send an ACK immediately
    delay(random(5, 30));
    uint8_t ackBuf[sizeof(AckPacket)];
    uint8_t ackLen = buildAck(mp, ackBuf);
    radio.send(ackBuf, ackLen);
    radio.startReceive();
    // ledPulse(10);

    // 2. If this is the Gateway, forward the packet upstream
    if (myRole == ROLE_GATEWAY) {
        forwardToEsp32(pkt);
    }

    // 3. Relay the packet if it did not originate from this device
    if (mp.originId != myNodeId) {
        relayPacket(mp);
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Actuator role: relay control, feedback reading, and status reporting
//  Assumes continuous power and continuous RX. Power-saving measures (such as STOP mode) are unnecessary.
// ─────────────────────────────────────────────────────────────────────────
static void actApplyRelay(uint8_t value) {
    actCommandedState = value;
    // [Requires on-site adjustment] Set HIGH/LOW according to the relay module drive polarity
    digitalWrite(RELAY_PIN, (actCommandedState & 0x01) ? HIGH : LOW);
}

// Return the raw voltage level of FEEDBACK_PIN (HIGH=1, LOW=0).
// The firmware does not interpret what HIGH means (ON or OFF);
// the receiver (e.g. MQTT/Home Assistant) determines this based on the wiring.
static uint8_t actReadFeedbackPin() {
    return (digitalRead(FEEDBACK_PIN) == HIGH) ? 1 : 0;
}

static void actSendStatus() {
    StatusPacket sp{};
    sp.magic0 = 0xB2; sp.magic1 = 0x60;
    sp.originId = myNodeId;
    sp.seq = actStatusSeq++;
    sp.commandedState = actCommandedState;
    sp.feedbackPinState = actFeedbackPinState;
    sp.lastCmdSeqApplied = actLastCmdSeqApplied;
    sp.crc8 = crc8((uint8_t*)&sp, sizeof(sp) - 1);

    radio.send((const uint8_t*)&sp, sizeof(sp));
    radio.startReceive(); // Return to continuous RX

    DBG("[ACT] status sent commanded=");
    DBG(actCommandedState);
    DBG(" feedback_pin=");
    DBGLN(actFeedbackPinState);
}

static void loopActuator() {
    // ── Receive downlink commands ──
    PacketInfo pkt;
    if (radio.checkForPacket(pkt) && !pkt.crcError && pkt.length == sizeof(CmdPacket)) {
        CmdPacket cp;
        memcpy(&cp, pkt.data, sizeof(cp));

        if (cp.magic0 == 0xC0 && cp.magic1 == 0x60 && cp.targetNodeId == myNodeId) {
            DBG("[ACT] cmd received type=");
            DBG(cp.cmdType);
            DBG(" value=");
            DBGLN(cp.cmdValue);

            if (cp.cmdType == 0) {
                actApplyRelay(cp.cmdValue);
            }
            actLastCmdSeqApplied = cp.cmdSeq;

            // // ledPulse(10);

            // Immediately after applying a command, allow for relay actuation delay
            // (the response time of a mechanical relay) before reading feedback again.
            delay(50);
            actFeedbackPinState = actReadFeedbackPin();
            actDebounceRawState = actFeedbackPinState;
            actDebounceSinceMs = millis();

            actSendStatus(); // Immediately report both the commanded and measured states
            actLastStatusSentMs = millis();
        }
    }

    // ── Continuously monitor feedback changes (detect state changes unrelated to commands) ──
    // Standard debounce: reset the timer when the raw reading changes,
    // and accept the value as stable if it remains unchanged for the required interval.
    uint8_t rawNow = actReadFeedbackPin();
    if (rawNow != actDebounceRawState) {
        actDebounceRawState = rawNow;
        actDebounceSinceMs = millis();
    } else if (actDebounceRawState != actFeedbackPinState &&
               millis() - actDebounceSinceMs >= ACTUATOR_FEEDBACK_DEBOUNCE_MS) {
        actFeedbackPinState = actDebounceRawState;

        DBG("[ACT] feedback pin changed unexpectedly -> ");
        DBGLN(actFeedbackPinState);

        // The measured state changed without a command (unexpected change, possibly due to a fault or external cause),
        // so report it immediately rather than waiting for the next periodic transmission.
        actSendStatus();
        actLastStatusSentMs = millis();
    }

    // ── Periodic status transmission (also serves as a heartbeat) ──
    if (millis() - actLastStatusSentMs >= ACTUATOR_STATUS_INTERVAL_MS) {
        actLastStatusSentMs = millis();
        actFeedbackPinState = actReadFeedbackPin();
        actSendStatus();
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Role-specific loops
// ─────────────────────────────────────────────────────────────────────────
static void sendMotionOrHeartbeat(bool isHeartbeat) {
    Wire.setSDA(PB9);
    Wire.setSCL(PB8);
    Wire.begin();

    DBG("[NODE] sending ");
    DBGLN(isHeartbeat ? "heartbeat" : "motion event");

    if (!isHeartbeat && sensorAvailable) {
        bmi160WakeGyro();
    }

    MeshPacket mp{};
    mp.magic0 = 0xB1; mp.magic1 = 0x60;
    mp.originId = myNodeId;
    mp.seq = mySeq++;
    mp.hopLimit = DEFAULT_HOP_LIMIT;
    mp.eventType = isHeartbeat ? 0 : 1;

    if (sensorAvailable) {
        mp.payloadLen = bmi160ReadMotionPayload(mp.payload);
    } else {
        mp.payloadLen = 0;
    }

    bool delivered = sendWithAck(mp);
    DBG("[NODE] delivered=");
    DBGLN(delivered ? "yes" : "no");

    if (!isHeartbeat && sensorAvailable) {
        bmi160SleepGyro();
    }
}

static void loopNode() {
    if (motionWakeFlag) {
        motionWakeFlag = false;
        if (sensorAvailable) bmi160ClearInterrupt();

        uint32_t now = millis();
        static uint32_t lastMotionSendMs = 0;
        if (now - lastMotionSendMs >= MOTION_COOLDOWN_MS) {
            lastMotionSendMs = now;
            sendMotionOrHeartbeat(false);
        }
    }

    sendMotionOrHeartbeat(true); // Heartbeat, including after waking due to an RTC alarm
    enterStopModeUntilNextEvent(HEARTBEAT_INTERVAL_SEC);
}

static void loopRepeater() {
    PacketInfo pkt;
    if (radio.checkForPacket(pkt)) handleIncomingMeshPacket(pkt);
}

static void loopGateway() {
    pollEsp32Downlink();

    PendingCmd c;
    if (cmdQueuePop(c)) {
        sendCommandToActuator(c.target, c.type, c.value);
    }

    PacketInfo pkt;
    if (radio.checkForPacket(pkt)) dispatchGatewayPacket(pkt);
}

/* static void mark(uint8_t n) {          // Flash n times
    pinMode(PB11, OUTPUT);
    for (uint8_t i = 0; i < n; i++) {
        digitalWrite(PB11, HIGH); delay(100);
        digitalWrite(PB11, LOW);  delay(150);
    }
    delay(400);
} */

// ─────────────────────────────────────────────────────────────────────────
//  Setup / Loop
// ─────────────────────────────────────────────────────────────────────────
void setup() {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    //mark(1);

#if MESH_DEBUG_SERIAL
    Serial1.begin(115200);
    delay(200);
    DBGLN("[INIT] mesh firmware starting...");
#endif

    //mark(2);

    Serial3.begin(115200);

    //mark(3);

    LowPower.begin();

    //mark(4);

    if (!radio.begin(RF_FREQUENCY_HZ)) {
        DBGLN("[ERR] radio init failed");
        // while (true) { digitalWrite(LED_PIN, HIGH); delay(100); digitalWrite(LED_PIN, LOW); delay(100); }
    }
    radio.setSpreadingFactor(RF_SF);
    radio.setBandwidth(SX1262Radio::bwFromKHz(RF_BW_KHZ));
    radio.setCodingRate(RF_CR);
    radio.setTxPower(RF_TX_POWER_DBM);

    myNodeId = deviceId16();
    myRole = determineRole();
    // blinkRolePattern(myRole); // 1 flash = Node, 2 = Repeater, 3 = Gateway, 4 = Actuator
    if (myRole != ROLE_GATEWAY) {
        Serial3.end();
        pinMode(PB11, OUTPUT);
        for (uint8_t i = 0; i <= myRole; i++) {   // NODE=1, REPEATER=2, ACTUATOR=4 flashes
            digitalWrite(PB11, HIGH); delay(150);   // Active HIGH
            digitalWrite(PB11, LOW);  delay(150);
        }
    }

    switch (myRole) {
        case ROLE_NODE: {
            Wire.setSDA(PB9);
            Wire.setSCL(PB8);
            Wire.begin();
            sensorAvailable = bmi160Present();

            if (sensorAvailable) {
                bmi160SetupAnyMotionInterrupt();
                pinMode(BMI160_INT_PIN, INPUT);
                LowPower.attachInterruptWakeup(BMI160_INT_PIN, onMotionWake, RISING, DEEP_SLEEP_MODE);
            } else {
                // Fail-safe behavior when the sensor is disconnected or faulty:
                // a floating INT1 line can cause false wake-ups,
                // so stabilize it with a pulldown and do not register the interrupt (continue with heartbeats only).
                pinMode(BMI160_INT_PIN, INPUT_PULLDOWN);
                // blinkErrorPattern(5); // Five flashes indicate a missing or faulty sensor
                DBGLN("[WARN] BMI160 not detected. Heartbeat-only fallback.");
            }
            radio.goStandby();
            break;
        }
        case ROLE_ACTUATOR: {
            pinMode(RELAY_PIN, OUTPUT);
            digitalWrite(RELAY_PIN, LOW); // Default to the safe state (OFF) at startup
            pinMode(FEEDBACK_PIN, INPUT_PULLUP);

            actFeedbackPinState = actReadFeedbackPin();
            actDebounceRawState = actFeedbackPinState;
            actDebounceSinceMs = millis();

            radio.startReceive(); // Assumes continuous power and RX; do not use STOP mode
            actSendStatus(); // Report the current state once at startup for initial Gateway synchronization
            actLastStatusSentMs = millis();
            break;
        }
        case ROLE_REPEATER:
        case ROLE_GATEWAY:
        default:
            radio.startReceive();
            break;
    }

    if (roleConfigError) {
        // blinkErrorPattern(8); // Eight flashes indicate a configuration error: both ROLE_PIN and ACTUATOR_PIN are grounded
        DBGLN("[WARN] ROLE_PIN and ACTUATOR_PIN both grounded. Falling back to NODE.");
    }

    DBGLN("[INIT] ready");
}

void loop() {
    // // ledService();
    switch (myRole) {
        case ROLE_NODE:      loopNode();      break;
        case ROLE_REPEATER:  loopRepeater();  break;
        case ROLE_GATEWAY:   loopGateway();   break;
        case ROLE_ACTUATOR:  loopActuator();  break;
    }
}
