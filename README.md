# LoRa Mesh Sensor & Relay Network (STM32 + ESP32-C6) 

![Lora_Network_MQTT_Control](./openremote_lora_mqtt.png)

A small 920 MHz LoRa mesh for battery sensors and relay control.
Every field device is an **STM32F103C8 (Blue Pill) + DX-LR30 (SX1262)** running one firmware;
an **ESP32-C6** bridges the mesh to an **MQTTS** broker (e.g. OpenRemote, Home Assistant).

```
Sensor Node ──LoRa──► Repeater ──LoRa──► Gateway (STM32 + ESP32-C6) ──MQTTS──► Broker
                                              ▲                                   │
                      Actuator (relay) ◄──LoRa┘◄──────── cmd/relay ───────────────┘
```

## Repository layout

```
.
├── README.md
├── gen_mesh_key.py    # Generates the shared relay-command key
├── esp32_gateway/     # ESP32-C6 gateway (ESP-IDF, PlatformIO)
└── stm32_dx-lr30/     # STM32F103C8 mesh firmware (Arduino/STM32duino, PlatformIO)
```

## STM32 firmware — `stm32_dx-lr30/`

One firmware image for all field devices. The role is detected at boot:

| Role | How it is selected | Purpose |
|---|---|---|
| Node | no jumpers | Low-power sensor, wakes on BMI160 motion or heartbeat |
| Repeater | PB0 → GND | Relays mesh packets |
| Actuator | PB1 → GND | Relay output with physical feedback input |
| Gateway | ESP32 answers UART PING | Forwards mesh traffic to the ESP32 and relay commands back |

Features: ACK + retry, hop-limited relaying with duplicate suppression, STOP-mode sleep on Nodes,
command queue and immediate status feedback on the Gateway/Actuator pair, CRC8-checked UART link,
LoRa payload CRC, and **authenticated relay commands** (SipHash-2-4 signature + Actuator-issued nonce).

Details, pin map and calibration checklist: [`stm32_dx-lr30/src/README.md`](stm32_dx-lr30/src/README.md)

```bash
python3 gen_mesh_key.py        # once, from the repository root: writes stm32_dx-lr30/src/mesh_secret.h
cd stm32_dx-lr30
pio run -e bluepill_mesh -t upload
```

The build fails without `src/mesh_secret.h` (it is git-ignored). The script also prints the same key as
32 hex characters, which you enter in the Gateway's setup portal.

## ESP32-C6 gateway — `esp32_gateway/`

- Built on **ESP-IDF** (the Arduino framework is not supported for ESP32-C6 on PlatformIO).
- Answers the STM32's PING/PONG handshake right after boot, so the STM32 detects the Gateway role.
- Publishes node data over **MQTTS (TLS, username/password)** and subscribes to relay commands.
- Signs each relay command (SipHash-2-4 over the Actuator's latest nonce) before passing it to the STM32.
- Wi-Fi/MQTT settings and the relay command key are entered through an **AP-mode setup portal**
  and stored in NVS (nothing is compiled in).

| Topic | Direction | Payload |
|---|---|---|
| `<prefix>/<node_id>/data` | publish | JSON (heartbeat / motion, RSSI, SNR, accel/gyro) |
| `<prefix>/<node_id>/status` | publish | JSON (`commanded_on`, `feedback_pin_high`, `last_cmd_seq_applied` = nonce of the applied command, ...) |
| `<prefix>/+/cmd/relay` | subscribe | `ON` / `1` = on, otherwise off |

Setup portal, Kconfig options and broker configuration: [`esp32_gateway/README.md`](esp32_gateway/README.md)

```bash
cd esp32_gateway
pio run -t menuconfig     # "Gateway Configuration"
pio run -t upload
```

## Wiring between the two MCUs

| STM32 | ESP32-C6 |
|---|---|
| PB10 (USART3 TX) | UART RX (set in Kconfig) |
| PB11 (USART3 RX) | UART TX (set in Kconfig) |
| GND | GND |

Do not connect the reset lines. The STM32 detects the Gateway by sending PING at boot, so the
ESP32 should start first (the STM32 retries 5 times).

## Relay command authentication

Relay commands (Gateway → Actuator) are signed and checked:

1. Every Actuator puts a single-use **nonce** in each of its status packets.
2. The ESP32 remembers the latest nonce per Actuator. For an MQTT command it signs
   `target + nonce + command` with **SipHash-2-4** (8-byte tag) and sends it to the STM32 over UART.
3. The STM32 Gateway only forwards the signed command over LoRa.
4. The Actuator applies it only if the tag verifies **and** the nonce is its current one, then issues a
   new nonce. Replayed or stale commands are never applied.

No clock and no stored sequence counter are needed. The key is a 16-byte secret shared by the ESP32
(entered in the setup portal, kept in NVS) and the STM32 firmware (`src/mesh_secret.h`, not committed).
Details: [`stm32_dx-lr30/src/README.md`](stm32_dx-lr30/src/README.md) and
[`esp32_gateway/README.md`](esp32_gateway/README.md).

## Important notes

- **Only relay commands are authenticated.** Nothing on the LoRa link is encrypted, and sensor data
  and Actuator status packets are not signed. Even so, do not use this for locks, safety-critical
  loads or unattended mains switching.
- **Regulations:** 920 MHz use is regulated (ARIB STD-T108 in Japan: certified module, carrier
  sense, transmit-time limits). Check the rules of your region.
- **Safety:** if you switch mains voltage with a relay, use proper isolation and a qualified
  installer.
- Boards with a CH340C auto-download circuit may fail to start after a reset when the PC serial
  port is closed. Power from a plain USB adapter, or power-cycle.
  
## Known limitations and future work

- **Only Gateway → Actuator relay commands are authenticated** (SipHash-2-4 + nonce). Sensor data
  (uplink) and Actuator status packets are not signed, so a third party in radio range could forge
  or replay them, and nothing is encrypted. A command that was signed but never delivered stays
  valid until the Actuator rotates its nonce (at most 60 s).
- If the ESP32 or the MQTT credentials are compromised, valid commands can be created. Protect the
  broker with an ACL and secure the Wi-Fi/portal. The key is stored in NVS in plain text (unless NVS
  encryption is enabled), and every STM32 contains it in flash because all devices run one firmware.
  Do not use it for locks, safety-critical loads or unattended mains switching.
- The Node→Gateway ACK is hop-by-hop (confirms delivery to the adjacent node), not end-to-end
  (delivery to the Gateway).
- Repeaters relay only sensor packets (`MeshPacket`). Relay commands and their status replies
  are not relayed, so an Actuator must be within direct range of the Gateway.
- Sensor Nodes cannot receive downlink commands (they listen only briefly after transmitting).
- The radio is half-duplex. Packets can be missed while the Gateway is transmitting or waiting
  for an Actuator's confirmation; Nodes recover through their own ACK/retry.
- Flooding is limited by `hopLimit`, but if the node count grows substantially, consider stronger
  broadcast-storm countermeasures (e.g. introducing a routing protocol).
- Right after an ESP32 reboot, or after a command that failed, the Actuator's nonce is unknown until
  its next status (up to 60 s); the command is held until then. Consecutive commands normally go
  through at once because each result status carries the next nonce.
- The nonce comes from timing, RSSI and ADC noise (the STM32F103 has no hardware RNG). It only has
  to not repeat, but its variation right after boot has not been measured.
- The LoRa payload CRC is enabled (`radio.setCrc(true)`), so corrupted packets are dropped. The
  application-level `crc8` field inside LoRa packets is computed but not verified on reception.
- Messages are published with QoS 0 and dropped while MQTT is disconnected; there is no
  store-and-forward buffering on the Gateway. Relay commands published while the Gateway is
  offline are lost, and no availability (Last Will) topic is published.
- The setup portal has no captive-portal DNS. Setup mode is entered on first boot, after a
  failed Wi-Fi connection at boot, or by a 5-second BOOT button press; it is **not** entered
  automatically when Wi-Fi is lost while running.

## License

MIT

## Credits

SX1262 driver based on https://github.com/NeutralSystem/STM32_DX-LR20_LORA
