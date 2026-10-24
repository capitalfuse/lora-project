#pragma once
// NVS configuration storage and AP-mode configuration portal

#include <stdbool.h>

struct GwConfig {
    bool valid;            // True if the SSID and broker URI are configured
    char ssid[33];
    char wifi_pass[65];
    char mqtt_uri[128];
    char mqtt_user[64];
    char mqtt_pass[64];
    char prefix[48];       // Topic prefix (must not start or end with /)
};

extern GwConfig g_cfg;

// Load g_cfg from NVS (valid=false if not configured; prefix defaults to the Kconfig value)
void gw_config_load(void);

// Check whether configuration mode has been requested
// (connection failure flag from the previous boot). The flag is consumed once.
bool gw_prov_requested(void);

// Record a request to enter configuration mode on the next boot
void gw_prov_request_next_boot(void);

// Start a task that reboots into configuration mode when the BOOT button is held for 5 seconds
void gw_prov_start_button_task(void);

// Start AP mode and the configuration web page.
// Does not return; the device reboots after saving the configuration or on timeout.
[[noreturn]] void gw_prov_run(void);