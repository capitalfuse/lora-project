// ============================================================================
//  ESP32-C6 Gateway firmware (ESP-IDF framework) - MQTTS-enabled version
//  Receives frames over UART from the STM32 (when ROLE_GATEWAY is detected in mesh_main.cpp),
//  publishes them over MQTTS (TLS), and returns a status byte to the STM32 (0x01=success / 0x02=failure).
//
//  Responds to the PING (0xF0) / PONG (0xF1) handshake immediately after startup,
//  allowing the STM32 to identify this device as the gateway.
//
//  Wi-Fi, MQTT, and UART pin settings can be changed in the
//  "Gateway Configuration" menu in `pio run -t menuconfig` (Kconfig.projbuild).
//
//  Changes for MQTTS support:
//   - Use mqtts://host:port as the broker URI (HAProxy terminates TLS on port 8884)
//   - Verify the server certificate (CA bundle or embedded CA certificate)
//   - Authenticate with a username and password (because allow_anonymous is false)
//   - Synchronize time using SNTP (the correct time is required to validate certificate expiration)
//   - Initialize UART before connecting to Wi-Fi so PING can be answered immediately after startup
// ============================================================================

#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <strings.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "nvs_flash.h"

#include "mqtt_client.h"
#include "esp_crt_bundle.h"
#include "driver/uart.h"
#include "cJSON.h"
#include "gw_prov.h"

#include "esp_timer.h"
#include "led_strip.h"

#define BLINK_GPIO          GPIO_NUM_8  // Onboard RGB LED on the ESP32-C6 DevKitC-1
#define LED_BLINK_MS        50          // LED on-time (ms)

static led_strip_handle_t s_led_strip = NULL;
static esp_timer_handle_t s_led_timer = NULL;

// ─────────────────────────────────────────────────────────────────────────
//  Timer callback that runs in the background after the interval and turns off the LED
// ─────────────────────────────────────────────────────────────────────────
static void led_timer_callback(void* arg) {
    if (s_led_strip) {
        led_strip_clear(s_led_strip);
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Initialize the LED and timer
// ─────────────────────────────────────────────────────────────────────────
static void async_led_init(void) {
    // Zero-initialize the structure with {} to prevent uninitialized warnings
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = BLINK_GPIO;
    strip_config.max_leds = 1;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT; // ★ Explicitly select the default clock
    rmt_config.resolution_hz = 10 * 1000 * 1000; // 10MHz

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip));
    led_strip_clear(s_led_strip);

    esp_timer_create_args_t timer_args = {};
    timer_args.callback = &led_timer_callback;
    timer_args.arg = NULL;
    timer_args.name = "led_off_timer";

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_led_timer));
}

// ─────────────────────────────────────────────────────────────────────────
//  Asynchronous LED activation function called when UART data is received
// ─────────────────────────────────────────────────────────────────────────
static void trigger_rx_led(uint8_t r, uint8_t g, uint8_t b) {
    if (!s_led_strip || !s_led_timer) return;

    // 1. Turn on the LED with the specified color
    led_strip_set_pixel(s_led_strip, 0, r, g, b);
    led_strip_refresh(s_led_strip);

    // 2. Stop the timer (reset it if it is already running), then restart it
    esp_timer_stop(s_led_timer);
    // The unit is microseconds (us), so multiply milliseconds by 1000
    esp_timer_start_once(s_led_timer, LED_BLINK_MS * 1000);
}

static const char* TAG = "gateway";

// UART link to the STM32 (defined here because the MQTT handler also uses it)
static const uart_port_t UART_PORT = (uart_port_t)CONFIG_GATEWAY_UART_PORT_NUM;
static const int UART_BUF_SIZE = 512;

#if CONFIG_GATEWAY_MQTT_USE_CUSTOM_CA
// platformio.ini: board_build.embed_txtfiles = src/ca_cert.pem
extern const char ca_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const char ca_cert_pem_end[]   asm("_binary_ca_cert_pem_end");
#endif


// ─────────────────────────────────────────────────────────────────────────
//  Wi-Fi
// ─────────────────────────────────────────────────────────────────────────
static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Returns true on successful connection, or false if connection fails within timeout_ms
static bool wifi_init_sta(uint32_t timeout_ms) {
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {};
    strlcpy((char*)wifi_config.sta.ssid, g_cfg.ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char*)wifi_config.sta.password, g_cfg.wifi_pass, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = g_cfg.wifi_pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to SSID:%s", g_cfg.ssid);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms));
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

// ─────────────────────────────────────────────────────────────────────────
//  Time synchronization (SNTP)
//  The correct time is required to validate the validity period of TLS server certificates.
//  Even if synchronization fails, SNTP continues running in the background,
//  so the MQTT client may connect later through automatic reconnection.
// ─────────────────────────────────────────────────────────────────────────
static void time_sync(void) {
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_GATEWAY_NTP_SERVER);
    ESP_ERROR_CHECK(esp_netif_sntp_init(&cfg));

    ESP_LOGI(TAG, "Waiting for SNTP sync (%s)...", CONFIG_GATEWAY_NTP_SERVER);
    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(30000)) == ESP_OK) {
        time_t now = 0;
        struct tm t = {};
        time(&now);
        gmtime_r(&now, &t);
        ESP_LOGI(TAG, "Time synced: %04d-%02d-%02d %02d:%02d:%02d UTC",
                 t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
    } else {
        ESP_LOGW(TAG, "SNTP sync timeout; TLS may fail until time is synced");
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  MQTT
// ─────────────────────────────────────────────────────────────────────────
static esp_mqtt_client_handle_t s_mqtt_client = nullptr;
static volatile bool s_mqtt_connected = false;

static uint8_t calc_crc8(const uint8_t* data, size_t len);
// Forward declaration for UART-related functionality (defined in the STM32 UART link section)
static void handle_relay_command(const char* topic, int topic_len, const char* data, int data_len) {
    char topicBuf[128] = {};
    int n = topic_len < (int)sizeof(topicBuf) - 1 ? topic_len : (int)sizeof(topicBuf) - 1;
    memcpy(topicBuf, topic, n);
    topicBuf[n] = '\0';

    char payloadBuf[16] = {};
    int m = data_len < (int)sizeof(payloadBuf) - 1 ? data_len : (int)sizeof(payloadBuf) - 1;
    memcpy(payloadBuf, data, m);
    payloadBuf[m] = '\0';

    char prefix[64];
    snprintf(prefix, sizeof(prefix), "%s/", g_cfg.prefix);
    size_t prefixLen = strlen(prefix);

    if (strncmp(topicBuf, prefix, prefixLen) != 0) return;
    if (strstr(topicBuf, "/cmd/relay") == nullptr) return;

    uint16_t nodeId = (uint16_t)atoi(topicBuf + prefixLen);
    uint8_t value = (strcasecmp(payloadBuf, "ON") == 0 || strcmp(payloadBuf, "1") == 0) ? 1 : 0;

    uint8_t frame[7];
    frame[0] = 0xCC; frame[1] = 0x33;
    frame[2] = nodeId & 0xFF;
    frame[3] = (nodeId >> 8) & 0xFF;
    frame[4] = 0;      // cmdType = 0 (relay control)
    frame[5] = value;
    frame[6] = calc_crc8(&frame[2], 4);   // target(2) + type + value
    uart_write_bytes(UART_PORT, (const char*)frame, sizeof(frame));

    ESP_LOGI(TAG, "relay cmd -> node %u = %u (topic=%s)", nodeId, value, topicBuf);
}

static void mqtt_event_handler(void* handler_args, esp_event_base_t base,
                                int32_t event_id, void* event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED: {
            ESP_LOGI(TAG, "MQTTS connected");
            s_mqtt_connected = true;
            char subTopic[80];
            snprintf(subTopic, sizeof(subTopic), "%s/+/cmd/relay", g_cfg.prefix);
            esp_mqtt_client_subscribe(s_mqtt_client, subTopic, 0);
            ESP_LOGI(TAG, "subscribed: %s", subTopic);
            break;
        }
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            s_mqtt_connected = false;
            break;
        case MQTT_EVENT_DATA:
            handle_relay_command(event->topic, event->topic_len, event->data, event->data_len);
            break;
        case MQTT_EVENT_ERROR:
            if (event->error_handle) {
                if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                    // Certificate validation failures, unsynchronized time, hostname mismatches, etc. are reported here
                    ESP_LOGE(TAG, "TLS/TCP error: esp_tls_err=0x%x tls_stack_err=0x%x sock_errno=%d",
                             event->error_handle->esp_tls_last_esp_err,
                             event->error_handle->esp_tls_stack_err,
                             event->error_handle->esp_transport_sock_errno);
                } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                    // For example, 5 = Not authorized (invalid username/password)
                    ESP_LOGE(TAG, "MQTT connection refused, return code=%d",
                             event->error_handle->connect_return_code);
                }
            }
            break;
        default:
            break;
    }
}

static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = g_cfg.mqtt_uri;   // Example: mqtts://mqtt.example.com:8884

    // Verify the server certificate
#if CONFIG_GATEWAY_MQTT_USE_CUSTOM_CA
    mqtt_cfg.broker.verification.certificate = ca_cert_pem_start;   // For a private CA or self-signed certificate
#else
    mqtt_cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;  // For public CAs such as Let's Encrypt
#endif

    // Authentication (Mosquitto: allow_anonymous false + password_file)
    mqtt_cfg.credentials.username = g_cfg.mqtt_user[0] ? g_cfg.mqtt_user : nullptr;
    mqtt_cfg.credentials.authentication.password = g_cfg.mqtt_pass[0] ? g_cfg.mqtt_pass : nullptr;

    mqtt_cfg.session.keepalive = 60;
    mqtt_cfg.network.reconnect_timeout_ms = 5000;
    mqtt_cfg.task.stack_size = 8192;   // TLS requires a larger stack

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(s_mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
}

// ─────────────────────────────────────────────────────────────────────────
//  UART link to the STM32
// ─────────────────────────────────────────────────────────────────────────
static void uart_link_init(void) {
    uart_config_t uart_config = {};
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity    = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, CONFIG_GATEWAY_UART_TX_PIN, CONFIG_GATEWAY_UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// Must match the MeshPacket definition in mesh_main.cpp
struct __attribute__((packed)) MeshPacket {
    uint8_t  magic0, magic1;
    uint16_t originId;
    uint32_t seq;
    uint8_t  hopLimit;
    uint8_t  eventType; // 0=heartbeat, 1=motion
    uint8_t  payloadLen;
    uint8_t  payload[56];
    uint8_t  crc8;
};

static uint8_t calc_crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

// Must match the StatusPacket definition in mesh_main.cpp
struct __attribute__((packed)) StatusPacket {
    uint8_t  magic0, magic1;
    uint16_t originId;
    uint32_t seq;
    uint8_t  commandedState;
    uint8_t  feedbackPinState; // Raw HIGH (1) / LOW (0) state of FEEDBACK_PIN
    uint32_t lastCmdSeqApplied;
    uint8_t  crc8;
};

enum ParseState { WAIT_AA, WAIT_55, WAIT_LEN1, WAIT_LEN2, WAIT_PAYLOAD, WAIT_CRC };

static void handle_complete_frame(const uint8_t* frameBuf, uint16_t payloadLen) {
    // frameBuf: [rssi(2)][snr(2)][loraLen(1)][loraPayload...]
    uint8_t result = 0x02; // Default: failure

    if (payloadLen >= 5) {
        int16_t rssi, snr;
        memcpy(&rssi, &frameBuf[0], 2);
        memcpy(&snr,  &frameBuf[2], 2);
        uint8_t loraLen = frameBuf[4];
        const uint8_t* loraPayload = &frameBuf[5];

        if (loraLen >= sizeof(MeshPacket) && loraPayload[0] == 0xB1 && loraPayload[1] == 0x60) {
            MeshPacket mp;
            memcpy(&mp, loraPayload, sizeof(MeshPacket));

            cJSON* doc = cJSON_CreateObject();
            cJSON_AddNumberToObject(doc, "node_id", mp.originId);
            cJSON_AddNumberToObject(doc, "seq", mp.seq);
            cJSON_AddStringToObject(doc, "event_type", mp.eventType == 0 ? "heartbeat" : "motion");
            cJSON_AddNumberToObject(doc, "rssi", rssi);
            cJSON_AddNumberToObject(doc, "snr", snr / 4.0);
            cJSON_AddNumberToObject(doc, "hop_limit", mp.hopLimit);

            if (mp.payloadLen >= 12) {
                int16_t ax, ay, az, gx, gy, gz;
                memcpy(&ax, &mp.payload[0], 2);
                memcpy(&ay, &mp.payload[2], 2);
                memcpy(&az, &mp.payload[4], 2);
                memcpy(&gx, &mp.payload[6], 2);
                memcpy(&gy, &mp.payload[8], 2);
                memcpy(&gz, &mp.payload[10], 2);
                cJSON_AddNumberToObject(doc, "ax", ax);
                cJSON_AddNumberToObject(doc, "ay", ay);
                cJSON_AddNumberToObject(doc, "az", az);
                cJSON_AddNumberToObject(doc, "gx", gx);
                cJSON_AddNumberToObject(doc, "gy", gy);
                cJSON_AddNumberToObject(doc, "gz", gz);
            }

            char* payloadStr = cJSON_PrintUnformatted(doc);

            char topic[80];
            snprintf(topic, sizeof(topic), "%s/%u/data", g_cfg.prefix, mp.originId);

            if (s_mqtt_connected) {
                int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, payloadStr, 0, 0, 0);
                if (msg_id >= 0) {
                    result = 0x01;
                    trigger_rx_led(0, 32, 0); // Green when MQTT publish succeeds
                } else {
                    trigger_rx_led(32, 0, 0); // Red when publish fails
                }
                ESP_LOGI(TAG, "-> %s %s (msg_id=%d)", topic, payloadStr, msg_id);
            } else {
                trigger_rx_led(32, 0, 0); // Red when MQTT is disconnected
                ESP_LOGW(TAG, "MQTT not connected, dropping message");
            }

            cJSON_free(payloadStr);
            cJSON_Delete(doc);
        } else if (loraLen >= sizeof(StatusPacket) && loraPayload[0] == 0xB2 && loraPayload[1] == 0x60) {
            StatusPacket sp;
            memcpy(&sp, loraPayload, sizeof(StatusPacket));

            cJSON* doc = cJSON_CreateObject();
            cJSON_AddNumberToObject(doc, "node_id", sp.originId);
            cJSON_AddNumberToObject(doc, "seq", sp.seq);
            cJSON_AddBoolToObject(doc, "commanded_on", (sp.commandedState & 0x01) != 0);
            cJSON_AddBoolToObject(doc, "feedback_pin_high", (sp.feedbackPinState & 0x01) != 0);
            cJSON_AddNumberToObject(doc, "last_cmd_seq_applied", sp.lastCmdSeqApplied);
            cJSON_AddNumberToObject(doc, "rssi", rssi);
            cJSON_AddNumberToObject(doc, "snr", snr / 4.0);

            char* payloadStr = cJSON_PrintUnformatted(doc);

            char topic[80];
            snprintf(topic, sizeof(topic), "%s/%u/status", g_cfg.prefix, sp.originId);

            if (s_mqtt_connected) {
                int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, payloadStr, 0, 0, 0);
                if (msg_id >= 0) {
                    result = 0x01;
                    trigger_rx_led(0, 32, 0); // Green when MQTT publish succeeds
                } else {
                    trigger_rx_led(32, 0, 0); // Red when publish fails
                }
                ESP_LOGI(TAG, "-> %s %s (msg_id=%d)", topic, payloadStr, msg_id);
            } else {
                trigger_rx_led(32, 0, 0); // Red when MQTT is disconnected
                ESP_LOGW(TAG, "MQTT not connected, dropping message");
            }

            cJSON_free(payloadStr);
            cJSON_Delete(doc);
        }
    }

    uart_write_bytes(UART_PORT, (const char*)&result, 1);
}

static void uart_task(void* pvParameters) {
    static uint8_t frameBuf[300];
    static uint16_t frameIdx = 0;
    static uint16_t payloadLen = 0;
    static uint8_t crcBuf[2 + 300];   // LEN(2) + payload
    ParseState state = WAIT_AA;

    uint8_t byteBuf;

    while (true) {
        int len = uart_read_bytes(UART_PORT, &byteBuf, 1, pdMS_TO_TICKS(1000));
        if (len <= 0) continue;

        uint8_t b = byteBuf;

        // Handle the gateway liveness handshake (PING/PONG) with highest priority
        if (state == WAIT_AA && b == 0xF0) {
            ESP_LOGI("UART", "PING (0xF0) received. Sending PONG");
            uint8_t pong = 0xF1;
            uart_write_bytes(UART_PORT, (const char*)&pong, 1);
            trigger_rx_led(32, 0, 32);
            continue;
        }

        switch (state) {
            case WAIT_AA:
                if (b == 0xAA) state = WAIT_55;
                break;
            case WAIT_55:
                state = (b == 0x55) ? WAIT_LEN1 : WAIT_AA;
                break;
            case WAIT_LEN1:
                payloadLen = b;
                crcBuf[0] = b;
                state = WAIT_LEN2;
                break;
            case WAIT_LEN2:
                payloadLen |= ((uint16_t)b << 8);
                crcBuf[1] = b;
                frameIdx = 0;
                state = (payloadLen <= sizeof(frameBuf)) ? WAIT_PAYLOAD : WAIT_AA;
                break;
            case WAIT_PAYLOAD:
                frameBuf[frameIdx] = b;
                crcBuf[2 + frameIdx] = b;  // Store data for CRC calculation
                frameIdx++;
                if (frameIdx >= payloadLen) state = WAIT_CRC;
                break;
            case WAIT_CRC:
                if (calc_crc8(crcBuf, 2 + payloadLen) == b) {
                    handle_complete_frame(frameBuf, payloadLen);
                } else {
                    ESP_LOGW(TAG, "uplink CRC mismatch, dropped");
                    uint8_t nack = 0x02;   // Allow the STM32 to stop waiting for a response sooner
                    uart_write_bytes(UART_PORT, (const char*)&nack, 1);
                }
                state = WAIT_AA;
                break;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Entry point
// ─────────────────────────────────────────────────────────────────────────
extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize UART first so PINGs from the STM32 can be answered during setup mode and while waiting for Wi-Fi/SNTP
    uart_link_init();
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
    gw_prov_start_button_task();   // Hold the BOOT button for 5 seconds to enter setup mode

    async_led_init(); // ★ Initialize the LED and timer
    // ★ Test: Turn the LED blue for 1 second immediately after startup
    led_strip_set_pixel(s_led_strip, 0, 0, 0, 32);
    led_strip_refresh(s_led_strip);
    vTaskDelay(pdMS_TO_TICKS(1000));
    led_strip_clear(s_led_strip);

    gw_config_load();
    bool force = gw_prov_requested();   // Wi-Fi connection failure flag from the previous boot (check and consume it first)
    if (!g_cfg.valid || force) {
        ESP_LOGW(TAG, "entering setup (AP) mode");
        gw_prov_run();   // Reboots after saving or on timeout (does not return)
    }

    if (!wifi_init_sta(30000)) {
        ESP_LOGW(TAG, "WiFi connect failed -> setup mode on next boot");
        gw_prov_request_next_boot();
        esp_restart();
    }
    time_sync();        // Synchronize the clock before starting MQTT to validate TLS certificates
    mqtt_app_start();

    ESP_LOGI(TAG, "gateway ready");
}
