#pragma once

#include "esp_err.h"
#include "data_bridge.h"
#include <stdbool.h>
#include <stdint.h>

typedef void (*rsc_conn_cb_t)(bool connected);
typedef void (*rsc_log_cb_t)(char level, const char *msg);

esp_err_t rsc_server_init(rsc_conn_cb_t conn_cb, rsc_log_cb_t log_cb);
esp_err_t rsc_server_start_advertising(void);
esp_err_t rsc_server_notify(const rsc_data_t *data);
bool rsc_server_is_connected(void);
bool rsc_server_notifications_enabled(void);

typedef struct {
    bool connected;
    bool notifications_enabled;
    uint32_t notify_attempts;
    uint32_t notify_success;
    uint32_t notify_skipped;
    uint32_t notify_errors;
    int last_error;
    uint32_t last_subscription_change_ms;
    uint32_t last_success_ms;
    uint16_t last_speed_256ths_mps;
    uint8_t last_cadence_spm;
    uint32_t last_distance_dm;
} rsc_diagnostics_t;

void rsc_server_get_diagnostics(rsc_diagnostics_t *out);
