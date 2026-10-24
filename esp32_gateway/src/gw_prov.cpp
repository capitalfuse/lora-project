// ============================================================================
//  gw_prov.cpp - NVS configuration storage + AP-mode configuration portal
// ============================================================================
#include "gw_prov.h"

#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cctype>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "nvs.h"

static const char* TAG = "prov";
static const char* NVS_NS = "gwcfg";
static const char* KEY_FORCE = "force_prov";

GwConfig g_cfg;

// ─────────────────────────────────────────────────────────────────────────
//  NVS
// ─────────────────────────────────────────────────────────────────────────
static void nvs_read_str(nvs_handle_t h, const char* key, char* out, size_t sz) {
    size_t len = sz;
    if (nvs_get_str(h, key, out, &len) != ESP_OK) out[0] = '\0';
}

void gw_config_load(void) {
    memset(&g_cfg, 0, sizeof(g_cfg));
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_read_str(h, "ssid",      g_cfg.ssid,      sizeof(g_cfg.ssid));
        nvs_read_str(h, "wpass",     g_cfg.wifi_pass, sizeof(g_cfg.wifi_pass));
        nvs_read_str(h, "mqtt_uri",  g_cfg.mqtt_uri,  sizeof(g_cfg.mqtt_uri));
        nvs_read_str(h, "mqtt_user", g_cfg.mqtt_user, sizeof(g_cfg.mqtt_user));
        nvs_read_str(h, "mqtt_pass", g_cfg.mqtt_pass, sizeof(g_cfg.mqtt_pass));
        nvs_read_str(h, "prefix",    g_cfg.prefix,    sizeof(g_cfg.prefix));
        nvs_close(h);
    }
    if (!g_cfg.prefix[0]) strlcpy(g_cfg.prefix, CONFIG_GATEWAY_MQTT_TOPIC_PREFIX, sizeof(g_cfg.prefix));
    g_cfg.valid = (g_cfg.ssid[0] != '\0') && (g_cfg.mqtt_uri[0] != '\0');
}

static esp_err_t config_save(const GwConfig& c) {
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_set_str(h, "ssid", c.ssid);
    if (e == ESP_OK) e = nvs_set_str(h, "wpass", c.wifi_pass);
    if (e == ESP_OK) e = nvs_set_str(h, "mqtt_uri", c.mqtt_uri);
    if (e == ESP_OK) e = nvs_set_str(h, "mqtt_user", c.mqtt_user);
    if (e == ESP_OK) e = nvs_set_str(h, "mqtt_pass", c.mqtt_pass);
    if (e == ESP_OK) e = nvs_set_str(h, "prefix", c.prefix);
    if (e == ESP_OK) {
        nvs_erase_key(h, KEY_FORCE);   // Clear the configuration-mode request flag (ignore if it does not exist)
        e = nvs_commit(h);
    }
    nvs_close(h);
    return e;
}

void gw_prov_request_next_boot(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, KEY_FORCE, 1);
        nvs_commit(h);
        nvs_close(h);
    }
}

bool gw_prov_requested(void) {
    bool req = false;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        uint8_t f = 0;
        if (nvs_get_u8(h, KEY_FORCE, &f) == ESP_OK && f) {
            req = true;
            nvs_erase_key(h, KEY_FORCE);   // One-shot flag
            nvs_commit(h);
        }
        nvs_close(h);
    }
    return req;
}

// ─────────────────────────────────────────────────────────────────────────
//  Monitor long press of the BOOT button (5 seconds)
//  Note: GPIO9 on the ESP32-C6 is a strapping pin. Pressing it after startup is fine.
// ─────────────────────────────────────────────────────────────────────────
static void button_task(void* arg) {
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << CONFIG_GATEWAY_PROV_BUTTON_GPIO;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io);

    int held = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (gpio_get_level((gpio_num_t)CONFIG_GATEWAY_PROV_BUTTON_GPIO) == 0) {
            if (++held >= 50) {
                ESP_LOGW(TAG, "button held 5s -> reboot into setup mode");
                gw_prov_request_next_boot();
                esp_restart();
            }
        } else {
            held = 0;
        }
    }
}

void gw_prov_start_button_task(void) {
    xTaskCreate(button_task, "prov_btn", 3072, NULL, 3, NULL);
}

// ─────────────────────────────────────────────────────────────────────────
//  String helpers
// ─────────────────────────────────────────────────────────────────────────
static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    return (tolower((unsigned char)c) - 'a') + 10;
}

static void url_decode(char* dst, size_t dstsz, const char* src, size_t n) {
    size_t o = 0;
    for (size_t i = 0; i < n && o + 1 < dstsz; i++) {
        if (src[i] == '+') {
            dst[o++] = ' ';
        } else if (src[i] == '%' && i + 2 < n && isxdigit((unsigned char)src[i + 1]) && isxdigit((unsigned char)src[i + 2])) {
            dst[o++] = (char)((hexval(src[i + 1]) << 4) | hexval(src[i + 2]));
            i += 2;
        } else {
            dst[o++] = src[i];
        }
    }
    dst[o] = '\0';
}

// Extract the value for a key from application/x-www-form-urlencoded data
static bool form_get(const char* body, const char* key, char* out, size_t outsz) {
    size_t klen = strlen(key);
    const char* p = body;
    while (*p) {
        const char* end = strchr(p, '&');
        size_t plen = end ? (size_t)(end - p) : strlen(p);
        if (plen > klen && strncmp(p, key, klen) == 0 && p[klen] == '=') {
            url_decode(out, outsz, p + klen + 1, plen - klen - 1);
            return true;
        }
        if (!end) break;
        p = end + 1;
    }
    out[0] = '\0';
    return false;
}

static void html_escape(char* dst, size_t dstsz, const char* src) {
    size_t o = 0;
    for (; *src && o + 7 < dstsz; src++) {
        switch (*src) {
            case '&':  o += snprintf(dst + o, dstsz - o, "&amp;");  break;
            case '<':  o += snprintf(dst + o, dstsz - o, "&lt;");   break;
            case '>':  o += snprintf(dst + o, dstsz - o, "&gt;");   break;
            case '"':  o += snprintf(dst + o, dstsz - o, "&quot;"); break;
            case '\'': o += snprintf(dst + o, dstsz - o, "&#39;");  break;
            default:   dst[o++] = *src;
        }
    }
    dst[o] = '\0';
}

static bool prefix_ok(const char* p) {
    size_t n = strlen(p);
    if (n == 0 || n > 47) return false;
    if (p[0] == '/' || p[n - 1] == '/') return false;
    if (strchr(p, '#') || strchr(p, '+')) return false;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────
//  Scan nearby SSIDs (for input suggestions)
// ─────────────────────────────────────────────────────────────────────────
#define MAX_SCAN 15
static char s_ssids[MAX_SCAN][33];
static int s_ssid_count = 0;
static wifi_ap_record_t s_recs[MAX_SCAN];

static void scan_ssids(void) {
    s_ssid_count = 0;
    if (esp_wifi_scan_start(nullptr, true) != ESP_OK) return;
    uint16_t n = MAX_SCAN;
    if (esp_wifi_scan_get_ap_records(&n, s_recs) != ESP_OK) return;
    for (int i = 0; i < n && s_ssid_count < MAX_SCAN; i++) {
        const char* s = (const char*)s_recs[i].ssid;
        if (!s[0]) continue;
        bool dup = false;
        for (int j = 0; j < s_ssid_count; j++) {
            if (strcmp(s_ssids[j], s) == 0) { dup = true; break; }
        }
        if (!dup) strlcpy(s_ssids[s_ssid_count++], s, sizeof(s_ssids[0]));
    }
    ESP_LOGI(TAG, "scan: %d SSIDs", s_ssid_count);
}

// ─────────────────────────────────────────────────────────────────────────
//  Web page
// ─────────────────────────────────────────────────────────────────────────
static volatile int64_t s_last_activity_us = 0;
static volatile bool s_saved = false;

static void touch(void) { s_last_activity_us = esp_timer_get_time(); }

static const char PAGE_HEAD[] =
"<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>LoRa Gateway Setup</title><style>"
"body{font-family:sans-serif;max-width:480px;margin:0 auto;padding:16px;background:#f5f5f5}"
"h1{font-size:1.3em}h2{font-size:1.05em;margin-top:1.4em;border-bottom:1px solid #ccc}"
"label{display:block;margin-top:.8em;font-weight:bold}"
"input{display:block;width:100%;box-sizing:border-box;padding:8px;margin-top:4px;font-size:1em}"
"small{color:#666}button{margin-top:1.5em;width:100%;padding:12px;font-size:1.1em}"
"</style></head><body><h1>LoRa Gateway 設定</h1>";

static void send_field(httpd_req_t* req, const char* label, const char* name, const char* type,
                       const char* value, int maxlen, const char* hint, const char* list = nullptr) {
    char esc[256];
    char line[640];
    html_escape(esc, sizeof(esc), value);
    snprintf(line, sizeof(line),
             "<label>%s<input type=\"%s\" name=\"%s\" value=\"%s\" maxlength=\"%d\"%s%s%s autocomplete=\"off\"></label>%s%s%s",
             label, type, name, esc, maxlen,
             list ? " list=\"" : "", list ? list : "", list ? "\"" : "",
             hint[0] ? "<small>" : "", hint, hint[0] ? "</small>" : "");
    httpd_resp_sendstr_chunk(req, line);
}

static esp_err_t root_get_handler(httpd_req_t* req) {
    touch();
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_sendstr_chunk(req, PAGE_HEAD);

    // SSID suggestions
    httpd_resp_sendstr_chunk(req, "<datalist id=\"ssids\">");
    for (int i = 0; i < s_ssid_count; i++) {
        char esc[160], opt[220];
        html_escape(esc, sizeof(esc), s_ssids[i]);
        snprintf(opt, sizeof(opt), "<option value=\"%s\">", esc);
        httpd_resp_sendstr_chunk(req, opt);
    }
    httpd_resp_sendstr_chunk(req, "</datalist><form method=\"post\" action=\"/save\"><h2>Wi-Fi</h2>");

    const char* keep = "Leave blank to keep the current value";
    send_field(req, "SSID", "ssid", "text", g_cfg.ssid, 32, "", "ssids");
    send_field(req, "Wi-Fi password", "wpass", "password", "", 64, g_cfg.ssid[0] ? keep : "");

    httpd_resp_sendstr_chunk(req, "<h2>MQTT</h2>");
    send_field(req, "Broker URI", "uri", "text",
               g_cfg.mqtt_uri[0] ? g_cfg.mqtt_uri : CONFIG_GATEWAY_MQTT_BROKER_URI, 127,
               "Example: mqtts://mqtt.example.com:8884");
    send_field(req, "Username", "muser", "text", g_cfg.mqtt_user, 63, "");
    send_field(req, "Password", "mpass", "password", "", 63, g_cfg.mqtt_user[0] ? keep : "");
    send_field(req, "Topic prefix", "prefix", "text", g_cfg.prefix, 47,
               "Example: lora (do not add / at the beginning or end)");

    httpd_resp_sendstr_chunk(req, "<button type=\"submit\">Save and reboot</button></form></body></html>");
    httpd_resp_sendstr_chunk(req, nullptr);
    return ESP_OK;
}

static esp_err_t send_msg_page(httpd_req_t* req, const char* status, const char* msg, bool back) {
    char esc[256], line[512];
    html_escape(esc, sizeof(esc), msg);
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_sendstr_chunk(req, PAGE_HEAD);
    snprintf(line, sizeof(line), "<p>%s</p>%s</body></html>", esc,
             back ? "<p><a href=\"/\">Back</a></p>" : "");
    httpd_resp_sendstr_chunk(req, line);
    httpd_resp_sendstr_chunk(req, nullptr);
    return ESP_OK;
}

static esp_err_t save_post_handler(httpd_req_t* req) {
    touch();
    if (req->content_len == 0 || req->content_len > 1024) {
        return send_msg_page(req, "400 Bad Request", "Invalid request", true);
    }
    char* body = (char*)malloc(req->content_len + 1);
    if (!body) return send_msg_page(req, "500 Internal Server Error", "Insufficient memory", true);

    size_t got = 0;
    while (got < req->content_len) {
        int r = httpd_req_recv(req, body + got, req->content_len - got);
        if (r <= 0) { free(body); return ESP_FAIL; }
        got += (size_t)r;
    }
    body[got] = '\0';

    GwConfig n = g_cfg;   // Leave password blank to keep the existing value.
    char tmp[128];
    form_get(body, "ssid", n.ssid, sizeof(n.ssid));
    if (form_get(body, "wpass", tmp, sizeof(tmp)) && tmp[0]) strlcpy(n.wifi_pass, tmp, sizeof(n.wifi_pass));
    form_get(body, "uri", n.mqtt_uri, sizeof(n.mqtt_uri));
    form_get(body, "muser", n.mqtt_user, sizeof(n.mqtt_user));
    if (form_get(body, "mpass", tmp, sizeof(tmp)) && tmp[0]) strlcpy(n.mqtt_pass, tmp, sizeof(n.mqtt_pass));
    form_get(body, "prefix", n.prefix, sizeof(n.prefix));
    free(body);

    const char* err = nullptr;
    if (!n.ssid[0]) {
        err = "Please enter an SSID";
    } else if (strncmp(n.mqtt_uri, "mqtts://", 8) != 0 || strlen(n.mqtt_uri) < 12) {
        err = "Broker URI should be like mqtts://www.example.com:8883";
    } else if (!prefix_ok(n.prefix)) {
        err = "Invalid topic prefix (must not be empty, start or end with '/', or contain '#' or '+').";
    }
    if (err) return send_msg_page(req, "400 Bad Request", err, true);

    if (config_save(n) != ESP_OK) {
        return send_msg_page(req, "500 Internal Server Error", "Failed to save configuration", true);
    }
    ESP_LOGI(TAG, "config saved (ssid=%s, uri=%s, user=%s, prefix=%s)", n.ssid, n.mqtt_uri, n.mqtt_user, n.prefix);

    send_msg_page(req, "200 OK", "Configuration saved. The gateway will reboot. You may close this page.", false);
    s_saved = true;
    return ESP_OK;
}

// Redirect unexpected URLs (such as smartphone connectivity checks) to the configuration page
static esp_err_t redirect_404(httpd_req_t* req, httpd_err_code_t err) {
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_send(req, nullptr, 0);
    return ESP_OK;
}

// ─────────────────────────────────────────────────────────────────────────
//  Main: access point + web server
// ─────────────────────────────────────────────────────────────────────────
[[noreturn]] void gw_prov_run(void) {
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();   // For SSID scanning

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);

    wifi_config_t ap = {};
    snprintf((char*)ap.ap.ssid, sizeof(ap.ap.ssid), "LoRa-GW-%02X%02X", mac[4], mac[5]);
    ap.ap.ssid_len = (uint8_t)strlen((char*)ap.ap.ssid);
    strlcpy((char*)ap.ap.password, CONFIG_GATEWAY_PROV_AP_PASSWORD, sizeof(ap.ap.password));
    ap.ap.channel = 1;
    ap.ap.max_connection = 2;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());

    scan_ssids();   // Run before clients connect (the AP may briefly become unstable during scanning)

    httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
    hc.stack_size = 8192;
    hc.lru_purge_enable = true;
    httpd_handle_t srv = nullptr;
    ESP_ERROR_CHECK(httpd_start(&srv, &hc));

    httpd_uri_t u_root = {};
    u_root.uri = "/";
    u_root.method = HTTP_GET;
    u_root.handler = root_get_handler;
    httpd_register_uri_handler(srv, &u_root);

    httpd_uri_t u_save = {};
    u_save.uri = "/save";
    u_save.method = HTTP_POST;
    u_save.handler = save_post_handler;
    httpd_register_uri_handler(srv, &u_save);

    httpd_register_err_handler(srv, HTTPD_404_NOT_FOUND, redirect_404);

    ESP_LOGW(TAG, "=== SETUP MODE ===");
    ESP_LOGW(TAG, "Wi-Fi: %s  / password: %s", (char*)ap.ap.ssid, CONFIG_GATEWAY_PROV_AP_PASSWORD);
    ESP_LOGW(TAG, "Open http://192.168.4.1/");

    touch();
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (s_saved) {
            vTaskDelay(pdMS_TO_TICKS(1500));   // Wait for the completion page to finish sending
            esp_restart();
        }
        if ((esp_timer_get_time() - s_last_activity_us) / 1000000 > CONFIG_GATEWAY_PROV_TIMEOUT_SEC) {
            ESP_LOGW(TAG, "setup mode idle timeout -> reboot");
            esp_restart();
        }
    }
}
