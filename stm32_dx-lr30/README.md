# STM32 DX-LR30 Mesh Firmware

A single-firmware LoRa mesh for sensors and relay control, built on
**STM32F103C8 (Blue Pill) + DX-LR30 (SX1262) + BMI160**, with an ESP32-C6 acting as the
MQTT gateway (see `../esp32_gateway/`).

Every device uses the **same Blue Pill board and the same firmware** (`bluepill_mesh`).
The role (Node / Repeater / Gateway / Actuator) is selected automatically at boot from
jumper wiring and whether an ESP32 is attached.

Original driver repository: https://github.com/NeutralSystem/STM32_DX-LR20_LORA

## Directory layout

```
stm32_dx-lr30/
│ ── lib/
│    　└── SX1262_Custom　　 # SX1262 driver (sx1262_driver.h/.cpp)
│         ├── README.md
│         ├── sx1262_driver.cpp
│         └── sx1262_driver.h
├── platformio.ini
└── src/
    ├── mesh_main.cpp         # Unified mesh firmware (main firmware)
    ├── mesh_protocol.h       # Packet structures, CRC8, device ID
    ├── cli_main.cpp          # Interactive LoRa CLI (RF tuning / ToA measurement)
    ├── lse_test.cpp          # LSE oscillator test
    └── BoardInfo.txt
```

`lib/SX1262_Custom/` is taken from the original repository. If it is not present in your
checkout, copy it to `lib/SX1262_Custom/`.

## Build

```bash
# CLI build (verification / RF tuning)
pio run -e bluepill_cli -t upload

# Production mesh firmware
pio run -e bluepill_mesh -t upload
```

Debug output goes to USART1 (PA9) at 115200 bps when `MESH_DEBUG_SERIAL=1` (default).
To disable it, add `-DMESH_DEBUG_SERIAL=0` to `build_flags` (it must be applied to the driver
as well as `mesh_main.cpp`). If the link then fails with `undefined reference to _write`, add:

```cpp
#if !MESH_DEBUG_SERIAL
extern "C" int _write(int file, char* ptr, int len) { (void)file; (void)ptr; return len; }
#endif
```

## Roles

| Role | Description |
|---|---|
| **Node** | Battery sensor. Sleeps in STOP mode, wakes on BMI160 Any-Motion (INT1) or on a periodic heartbeat, sends with ACK + retry. |
| **Repeater** | Always-on relay of `MeshPacket` frames (hop-limited, duplicate-suppressed). |
| **Gateway** | Receives mesh traffic, ACKs it, and forwards it to the ESP32 over UART. Also forwards MQTT relay commands to Actuators. |
| **Actuator** | Always-on relay output with a physical feedback input. Reports state back to the Gateway. |

### Role detection

PB0 and PB1 use the internal pull-ups only. No external resistors are needed.

| PB0 | PB1 | Role |
|---|---|---|
| open | open | **Node** (default) |
| GND | open | **Repeater** |
| open | GND | **Actuator** |
| GND | GND | Configuration error → falls back to Node |

If an ESP32 answers the UART PING (`0xF0`) with PONG (`0xF1`) within the retry window
(5 attempts × 300 ms), the device becomes **Gateway**, regardless of the jumpers.
A device with no ESP32 therefore takes up to 1.5 s longer to boot.

## Wiring

All roles share the same board. Only the parts relevant to a role need to be connected.

| Signal | STM32 pin | Connected to |
|---|---|---|
| SPI1 SCK/MISO/MOSI | PA5 / PA6 / PA7 | DX-LR30 |
| SX1262 NSS | PA4 | DX-LR30 |
| SX1262 RESET / BUSY | PA3 / PA2 | DX-LR30 |
| SX1262 TXEN / RXEN | PA0 / PA1 | DX-LR30 RF switch |
| USART1 (debug) | PA9 (TX) / PA10 (RX) | USB-serial adapter |
| USART3 (ESP32 link) | PB10 (TX) / PB11 (RX) | ESP32-C6 UART (cross TX/RX, common GND) |
| I2C1 (remapped) | PB8 (SCL) / PB9 (SDA) | BMI160 (Node only) |
| BMI160 INT1 | PB3 | BMI160 INT1 (Node only) |
| Role jumper 1 | PB0 | GND = Repeater |
| Role jumper 2 | PB1 | GND = Actuator |
| `RELAY_PIN` | PB4 | Relay drive input (Actuator only) |
| `FEEDBACK_PIN` | PB5 | Physical state feedback, internal pull-up (Actuator only) |

Notes:

- PB3/PB4 are shared with JTAG on the F103. With STM32duino only SWD is enabled by default, so
  they work as GPIO; if they do not, disable JTAG explicitly.
- PB11 (USART3 RX) is also wired to the on-board LED on some boards. The status LED is therefore
  **not used** by default. Do not rely on it on a Gateway.
- Power the BMI160 from the module's 3.3 V pin (not VIN).

## Radio settings

920 MHz, SF9, BW 125 kHz, CR 4/5, +13 dBm. **All devices must use identical RF settings.**
Check the regulations of your region before use (ARIB STD-T108 in Japan; duty cycle,
carrier sense and transmit-time limits apply). Use a certified module with its certified antenna.

Key timing constants in `mesh_main.cpp`:

| Constant | Default | Note |
|---|---|---|
| `HEARTBEAT_INTERVAL_SEC` | 3600 | Use a short value (e.g. 10) only while testing |
| `ACK_TIMEOUT_MS` | 150 | Re-tune after measuring ToA |
| `MAX_RETRIES` | 3 | Node → mesh |
| `CMD_MAX_RETRIES` | 3 | Gateway → Actuator |
| `CMD_CONFIRM_TIMEOUT_MS` | 300–400 | Wait for the Actuator's StatusPacket |
| `DEFAULT_HOP_LIMIT` | 3 | |

## Relay control (Gateway → Actuator)

```
MQTT publish "<prefix>/<id>/cmd/relay": "ON" / "OFF"
  → ESP32 (subscribed) → UART downlink frame → STM32 Gateway (command queue)
  → LoRa CmdPacket (targetNodeId)
  → Actuator applies it and answers immediately with a StatusPacket
  → Gateway confirms by cmdSeq; retries up to CMD_MAX_RETRIES

Actuator → StatusPacket (periodic, on command, or when FEEDBACK_PIN changes)
  → Gateway → ESP32 → MQTT "<prefix>/<id>/status"
     { commanded_on, feedback_pin_high, last_cmd_seq_applied, rssi, snr }
```

### Commanded state vs. feedback pin

`StatusPacket` carries two independent values:

- `commandedState`: what the Gateway asked for (the GPIO output level)
- `feedbackPinState`: the raw HIGH/LOW level of `FEEDBACK_PIN`

The firmware never interprets whether HIGH means ON. Give it meaning on the MQTT side according
to your wiring. A mismatch between the two reveals relay faults, wiring problems or a
misbehaving load. `FEEDBACK_PIN` is debounced (200 ms) and monitored continuously; an
unexpected change is reported immediately without waiting for the periodic status.

### Gateway behaviour while talking to an Actuator

- MQTT commands are **queued** (4 entries) and executed one at a time from the main loop,
  which avoids re-entrancy while UART and radio are both being serviced.
- While waiting for the Actuator's confirmation, packets from other devices are still
  handled (ACKed and forwarded) instead of being dropped.
- The radio is half-duplex. Packets that arrive while the Gateway is transmitting, or during
  retry back-off delays, can still be missed; Nodes recover through their own ACK/retry.

### Things to adapt on site

- `RELAY_PIN` polarity for your relay module (HIGH = ON is assumed).
- `FEEDBACK_PIN` wiring (flow switch, relay auxiliary contact, load status output, ...).
- Use opto-isolation for anything above 3.3 V; never connect mains signals directly.

## UART link to the ESP32

| Direction | Frame | CRC8 (poly 0x07) |
|---|---|---|
| STM32 → ESP32 | `AA 55 LEN(2) RSSI(2) SNR(2) LoRaLen LoRaPayload CRC` | over `LEN … payload` |
| ESP32 → STM32 | `CC 33 targetId(2) cmdType cmdValue CRC` | over `targetId … cmdValue` |
| ESP32 → STM32 | single byte `01` (published) / `02` (failed) | – |

Both firmwares must be updated together when the frame format changes.

## Before production use

| Item | What to do |
|---|---|
| ToA measurement | In the `bluepill_cli` environment, send data of the same length as `MeshPacket`/`AckPacket` with `sendhex` and record the TX time; adjust `ACK_TIMEOUT_MS`. |
| BMI160 I2C address | Confirm 0x68 or 0x69 with an I2C scanner (`BMI160_ADDR`). |
| BMI160 Any-Motion registers | Verify threshold, duration and bit layout against the Bosch datasheet (`bmi160SetupAnyMotionInterrupt()`). |
| Gyro in heartbeats | The gyro is suspended outside motion events, so heartbeat gyro values may be zero or stale. |
| RF regulations | Confirm TX power, duty cycle and carrier-sense rules for your region. |
| Heartbeat interval | Set `HEARTBEAT_INTERVAL_SEC` back to 3600 after testing. |
| Current consumption | Measure the Node with an ammeter; the BMI160 interrupt setup alone may not reach the expected low current. |

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Does not start after pressing reset while the PC serial port is closed | Boards with a CH340C auto-download circuit drive NRST/BOOT0 from DTR/RTS. Keep the terminal open, or power the board from a plain USB adapter. Power-cycling always works. |
| Node never publishes | Heartbeat may simply be 3600 s away. Reset the Node and check for the boot heartbeat. |
| Duplicate MQTT messages | Check for duplicated publish blocks in the ESP32 firmware. |
| `command NOT confirmed` | Check Actuator jumpers (PB1 = GND, PB0 open, no ESP32), RF settings and distance. |
| Gateway not detected | The ESP32 UART must be initialised before the STM32 sends PING; PING is retried 5 times. |

## License

MIT