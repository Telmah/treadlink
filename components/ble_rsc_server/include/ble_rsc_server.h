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
void rsc_server_get_diag_counters(uint32_t *attempts, uint32_t *sent,
                                  uint32_t *skipped, uint32_t *errors);
