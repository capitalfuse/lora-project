# STM32 DX-LR30 Mesh Sensor Network

A LoRa mesh sensor network built from STM32F103C8 + DX-LR30 (SX1262) + BMI160 + ESP32-C6,
with every board able to act as Node, Repeater or Gateway. The Gateway forwards mesh traffic
to an MQTT broker over **MQTTS (TLS)** and is configured through a **Wi-Fi setup portal (AP mode)**.

Original repository: https://github.com/NeutralSystem/STM32_DX-LR20_LORA

## Directory layout

```
esp32_gateway/                     # Standalone PlatformIO project for ESP32-C6 (ESP-IDF)
├── CMakeLists.txt
├── README.md
├── dependencies.lock
├── esp32_gateway.code-workspace
├── partitions.csv
├── platformio.ini
├── sdkconfig.esp32-c6-devkitc-1  # TLS certificate bundle, main task stack size
└── src
    ├── CMakeLists.txt          # Component dependencies
    ├── Kconfig.projbuild       # Build-time options (UART pins, defaults, setup AP, ...)
    ├── gw_prov.cpp             # NVS config storage + AP-mode setup portal (web form)
    ├── gw_prov.h
    ├── idf_component.yml
    └── main.cpp                # Wi-Fi (STA), SNTP, MQTTS publish/subscribe, UART bridge to STM32
```

## ESP32-C6 Gateway

The official PlatformIO `espressif32` platform supports ESP32-C6 with the **ESP-IDF framework
only** (the Arduino framework is not supported; see `platformio/platform-espressif32` issue
\#1694), so `esp32_gateway` is implemented on ESP-IDF from the start.

The ESP32-C6 has a built-in USB Serial/JTAG, so a single USB-C cable covers flashing, the serial
monitor and JTAG debugging (VS Code + PlatformIO Debug). No external JTAG probe is needed.

### What the Gateway does

- Answers the STM32's PING (0xF0) with PONG (0xF1) right after boot, so the STM32 detects the
  Gateway role. The UART is initialized **before** Wi-Fi/SNTP, so PING is answered even while the
  Gateway is still connecting or in setup mode.
- Receives frames from the STM32 over UART and publishes them to MQTT over TLS.
  Returns a result byte to the STM32 (`0x01` = success, `0x02` = failure).
- Subscribes to relay commands and forwards them to the STM32 over UART.

### MQTT topics

`<prefix>` is the topic prefix configured in the setup portal (default from Kconfig).

| Direction | Topic | Payload |
|---|---|---|
| Publish | `<prefix>/<node_id>/data` | JSON: `node_id`, `seq`, `event_type` (`heartbeat`/`motion`), `rssi`, `snr`, `hop_limit`, and `ax..gz` when present |
| Publish | `<prefix>/<node_id>/status` | JSON: `node_id`, `seq`, `commanded_on`, `feedback_pin_high`, `last_cmd_seq_applied`, `rssi`, `snr` |
| Subscribe | `<prefix>/+/cmd/relay` | `ON` or `1` = on, anything else = off |

Messages are published with QoS 0. The prefix must not start or end with `/`, must not contain
`#` or `+`, and must be at most 47 characters.

### Configuration: setup portal (AP mode)

Wi-Fi and MQTT settings are **not** compiled in. They are entered through a web form served by
the Gateway itself and stored in NVS.

**Entering setup mode**

| Trigger | Behavior |
|---|---|
| No saved configuration (first boot) | Starts in setup mode |
| Wi-Fi connection fails for 30 s at boot | Reboots into setup mode on the next boot |
| BOOT button (default GPIO9) held for 5 s while running | Reboots into setup mode |

Losing Wi-Fi *while running* does not trigger setup mode; the Gateway keeps retrying forever.

**Using the portal**

1. Connect a phone/PC to the Wi-Fi network `LoRa-GW-XXXX` (`XXXX` = last 2 bytes of the SoftAP MAC).
   The password is `CONFIG_GATEWAY_PROV_AP_PASSWORD` (default `gateway-setup` — **change it**).
2. Open `http://192.168.4.1/` in a browser. There is no captive-portal DNS, so the address must
   be opened manually. If the phone warns that the network has no internet access, choose to stay
   connected.
3. Fill in the form and press **Save and reboot**. The Gateway restarts in STA mode.

| Field | Notes |
|---|---|
| SSID | Nearby networks are offered as suggestions (scanned when setup mode starts) |
| Wi-Fi password | Leave blank to keep the current value. Empty password + no saved value = open network |
| Broker URI | Must start with `mqtts://`, e.g. `mqtts://mqtt.example.com:8884` |
| MQTT username / password | Password left blank = keep the current value |
| Topic prefix | See the topic rules above |

If nobody touches the portal for `CONFIG_GATEWAY_PROV_TIMEOUT_SEC` (default 300 s), the Gateway
reboots and tries STA mode again. This keeps it recovering by itself after a power outage while
the router is still down.

### Build-time options (Kconfig)

```bash
cd project/esp32_gateway
pio run -t menuconfig
# "Gateway Configuration" menu
```

| Option | Purpose |
|---|---|
| `GATEWAY_UART_PORT_NUM`, `GATEWAY_UART_TX_PIN`, `GATEWAY_UART_RX_PIN` | UART link to the STM32 |
| `GATEWAY_MQTT_BROKER_URI` | Default shown in the setup form |
| `GATEWAY_MQTT_TOPIC_PREFIX` | Default prefix shown in the setup form |
| `GATEWAY_NTP_SERVER` | NTP server for time sync (default `ntp.nict.jp`) |
| `GATEWAY_MQTT_USE_CUSTOM_CA` | Embed `ca_cert.pem` instead of using the public CA bundle |
| `GATEWAY_PROV_AP_PASSWORD` | Setup AP password (8+ characters) |
| `GATEWAY_PROV_BUTTON_GPIO` | Button for the 5 s long press (default 9) |
| `GATEWAY_PROV_TIMEOUT_SEC` | Setup mode idle timeout (default 300) |

The old `GATEWAY_WIFI_SSID`, `GATEWAY_WIFI_PASSWORD`, `GATEWAY_MQTT_USERNAME` and
`GATEWAY_MQTT_PASSWORD` options are no longer used and can be removed.

On the ESP32-C6, GPIO9 is a strapping pin. Pressing it *after* boot is fine, but holding it low
during reset enters the ROM download mode.

### `sdkconfig.esp32-c6-devkitc-1`

```
CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=y
CONFIG_MBEDTLS_CERTIFICATE_BUNDLE_DEFAULT_FULL=y
CONFIG_ESP_MAIN_TASK_STACK_SIZE=6144
```

The first two enable the public CA bundle used to verify Let's Encrypt (and other public CA)
certificates. PlatformIO reports a "Default Value Mismatch" notice for the stack size; this is
informational only.

### `src/CMakeLists.txt`

Pass each source file as a **separate** quoted argument:

```cmake
idf_component_register(
    SRCS "main.cpp" "gw_prov.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        driver
        esp_http_server
        esp_timer
        led_strip
)
```

If you use a custom CA, also add `board_build.embed_txtfiles = src/ca_cert.pem` to
`esp32_gateway/platformio.ini`.

## MQTTS and broker setup

Transport security and user authentication are independent:

- **TLS** (server certificate): lets the Gateway verify the broker and encrypts the link. With a
  public CA such as Let's Encrypt, the ESP32 verifies the certificate against its built-in CA
  bundle — no certificate file has to be stored on the device, and certificate renewals need no
  firmware update.
- **User authentication**: the Gateway sends the MQTT username/password inside the TLS session.
  No client certificate is required.

Because the TLS handshake validates the certificate's validity period, the Gateway syncs time via
SNTP before starting MQTT. The broker host name in the URI must match the certificate's CN/SAN
(an IP address will not verify).

### Recommended topology: TLS terminated at HAProxy

```
ESP32-C6 --TLS 8884--> HAProxy (TLS termination) --plain 1883--> Mosquitto (password + ACL)
```

Example for the OpenRemote proxy, based on its existing `listen mqtt` block. Add this block to
your own copy of `haproxy.cfg` and keep the existing `8883` block untouched:

```haproxy
listen mosquitto_mqtts
    bind *:8884 ssl crt /etc/haproxy/certs crt "${CERT_DIR}" crt "${CUSTOM_CERT_DIR}" no-tls-tickets
    mode tcp
    option clitcpka
    timeout client 3h
    timeout server 3h
    option logasap
    balance leastconn
    server mosquitto mosquitto:1883 resolvers docker_resolver
```

```yaml
services:
  proxy:
    ports:
      - "8884:8884"
    volumes:
      - ./haproxy.cfg:/data/proxy/haproxy.cfg
    environment:
      HAPROXY_CONFIG: '/data/proxy/haproxy.cfg'
```

Do not publish Mosquitto's port 1883 on the host; it should be reachable only through the proxy
on the Docker network.

### Mosquitto

```conf
listener 1883 0.0.0.0
protocol mqtt

allow_anonymous false
password_file /mosquitto/config/passwd
acl_file /mosquitto/config/acl.conf
```

Create a dedicated user for the Gateway (omit `-c` when the password file already exists) and
restrict it with an ACL. Replace `lora` with your topic prefix:

```bash
docker compose exec mosquitto mosquitto_passwd -b /mosquitto/config/passwd gw01 'your-password'
```

```conf
# /mosquitto/config/acl.conf
user gw01
topic write lora/+/data
topic write lora/+/status
topic read  lora/+/cmd/relay
```

Once `acl_file` is enabled, users not listed in it cannot read or write anything; add entries for
any other clients using the broker.

Connection test from a PC:

```bash
mosquitto_pub -h mqtt.example.com -p 8884 --capath /etc/ssl/certs \
  -u gw01 -P 'your-password' -t lora/1/cmd/relay -m ON
```

### Troubleshooting the Gateway connection

The serial log prints the cause on `MQTT_EVENT_ERROR`:

| Log | Likely cause |
|---|---|
| `TLS/TCP error: esp_tls_err=... tls_stack_err=...` | Certificate verification failed, time not synced, host name mismatch, or port 8884 unreachable |
| `MQTT connection refused, return code=5` | Wrong username or password |
| Connects, but messages never arrive | Possibly denied by the ACL (ACL denials raise no error; publishes are silently dropped). Check that the ACL topics match the prefix |
| No `Time synced` line | SNTP failed; a certificate may be treated as "not yet valid" until time is set |

## Build (STM32)

```bash
# CLI debug build (verification / RF tuning)
pio run -e bluepill_cli -t upload

# Production mesh firmware
pio run -e bluepill_mesh -t upload
```

## Wiring

| Signal | STM32 pin | Connected to |
|---|---|---|
| USART3 (ESP32 link) | PB10 (TX) / PB11 (RX) | ESP32-C6 UART (pins set via Kconfig; cross TX/RX) |

## Role detection

ESP32 answers the UART PING (0xF0) / PONG (0xF1) handshake

## Before production use

| Item | What to do | Where |
|---|---|---|
| Gateway Wi-Fi / MQTT settings | Enter SSID, Wi-Fi password, `mqtts://` broker URI, MQTT credentials and topic prefix through the setup portal; change the default setup AP password | `esp32_gateway` (`gw_prov.cpp`, Kconfig) |
| Broker user and ACL | Create a dedicated Mosquitto user for the Gateway and restrict it with an ACL that matches the topic prefix | Mosquitto `passwd` / `acl.conf` |

## Security notes

- Wi-Fi and MQTT passwords are stored in NVS in **plain text** unless NVS encryption is enabled.
  Consider enabling it if the device can be physically accessed.
- The setup portal uses plain HTTP, protected only by the WPA2 setup AP. Change
  `CONFIG_GATEWAY_PROV_AP_PASSWORD` from its default.
- Do not commit `sdkconfig` to version control if it contains credentials.
- Only `mqtts://` URIs are accepted by the setup form. Relax the check in `save_post_handler()`
  only if you deliberately need plain MQTT on a trusted network.
- ESP8266 is not recommended as a replacement for the Gateway: TLS needs 20–30 KB of heap, the
  handshake is slow, and TLS 1.3 is unsupported. Use ESP32-class chips for MQTTS.

## License

MIT