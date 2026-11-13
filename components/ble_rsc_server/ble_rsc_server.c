#include "ble_rsc_server.h"
#include "diag_trace.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_store.h"
#include "host/ble_sm.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

static const char *TAG = "rsc_server";

static rsc_conn_cb_t s_conn_cb;
static rsc_log_cb_t s_log_cb;
static uint16_t s_conn_handle;
static bool s_connected;
static bool s_notifications_enabled;
static uint16_t s_rsc_measurement_handle;
static bool s_first_notify_logged;

static void rsc_log(char level, const char *fmt, ...)
{
    if (!s_log_cb) return;
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    s_log_cb(level, buf);
}

// RSC Feature: total distance (bit 1) + walk/run status (bit 2)
static const uint16_t RSC_FEATURE_FLAGS = 0x0006;

// Sensor location: Top of shoe
static const uint8_t SENSOR_LOCATION = 0x01;

// GATT access callback for RSC characteristics
static int rsc_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return 0;

    switch (uuid16) {
    case 0x2A54: // RSC Feature
        os_mbuf_append(ctxt->om, &RSC_FEATURE_FLAGS, sizeof(RSC_FEATURE_FLAGS));
        break;
    case 0x2A5D: // Sensor Location
        os_mbuf_append(ctxt->om, &SENSOR_LOCATION, sizeof(SENSOR_LOCATION));
        break;
    }

    return 0;
}

// Device Information Service
static const char *DIS_MANUFACTURER = "TreadLink";
static const char *DIS_MODEL = "TL-1";
static const char *DIS_FW_REV = "1.0.0";

static int dis_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    const char *val = NULL;

    switch (uuid16) {
    case 0x2A29: val = DIS_MANUFACTURER; break;
    case 0x2A24: val = DIS_MODEL; break;
    case 0x2A26: val = DIS_FW_REV; break;
    }

    if (val && ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        os_mbuf_append(ctxt->om, val, strlen(val));
    }
    return 0;
}

// GATT service definitions
static const struct ble_gatt_svc_def rsc_svcs[] = {
    {
        // Device Information Service (0x180A)
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180A),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0x2A29), // Manufacturer Name
                .access_cb = dis_chr_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A24), // Model Number
                .access_cb = dis_chr_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A26), // Firmware Revision
                .access_cb = dis_chr_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {0},
        },
    },
    {
        // RSC Service (0x1814)
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x1814),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0x2A53), // RSC Measurement
                .access_cb = rsc_chr_access,
                .val_handle = &s_rsc_measurement_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A54), // RSC Feature
                .access_cb = rsc_chr_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A5D), // Sensor Location
                .access_cb = rsc_chr_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {0},
        },
    },
    {0},
};

// Capture GAP callback order and RSC state before/after each callback.
// The event-specific payload is kept in the existing 8-byte diagnostic data field.
static uint32_t rsc_diag_state_flags(void)
{
    return (s_connected ? 1U : 0U) | (s_notifications_enabled ? 2U : 0U);
}

static void rsc_diag_gap_callback(uint8_t trace_type, const struct ble_gap_event *event)
{
    uint8_t detail[8] = {0};
    uint8_t detail_len = 0;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        detail[0] = (uint8_t)event->connect.status;
        detail[1] = (uint8_t)(event->connect.conn_handle & 0xFF);
        detail[2] = (uint8_t)(event->connect.conn_handle >> 8);
        detail_len = 3;
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        detail[0] = event->disconnect.reason;
        detail[1] = (uint8_t)(s_conn_handle & 0xFF);
        detail[2] = (uint8_t)(s_conn_handle >> 8);
        detail_len = 3;
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        detail[0] = event->subscribe.cur_notify ? 1 : 0;
        detail[1] = event->subscribe.cur_indicate ? 1 : 0;
        detail[2] = (uint8_t)(event->subscribe.attr_handle & 0xFF);
        detail[3] = (uint8_t)(event->subscribe.attr_handle >> 8);
        detail[4] = (uint8_t)(event->subscribe.conn_handle & 0xFF);
        detail[5] = (uint8_t)(event->subscribe.conn_handle >> 8);
        detail_len = 6;
        break;
    case BLE_GAP_EVENT_ENC_CHANGE:
        detail[0] = (uint8_t)event->enc_change.status;
        detail[1] = (uint8_t)(event->enc_change.conn_handle & 0xFF);
        detail[2] = (uint8_t)(event->enc_change.conn_handle >> 8);
        detail_len = 3;
        break;
    default:
        break;
    }

    diag_trace_record(trace_type, (uint32_t)event->type, s_conn_handle,
                      rsc_diag_state_flags(), 0, detail, detail_len);
}

// GAP event handler
static int rsc_gap_event(struct ble_gap_event *event, void *arg)
{
    rsc_diag_gap_callback(DIAG_EVT_GAP_CALLBACK_ENTER, event);
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_connected = true;
            s_notifications_enabled = false;
            diag_trace_record(DIAG_EVT_GARMIN_CONNECT, s_conn_handle, 0, 0, 0, NULL, 0);
            rsc_log('I', "Garmin connected");
            if (s_conn_cb) s_conn_cb(true);
        } else {
            rsc_server_start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT: {
        uint8_t reason = event->disconnect.reason;
        const char *reason_str = "unknown";
        switch (reason) {
            case 0x13: reason_str = "remote terminated"; break;
            case 0x16: reason_str = "local terminated"; break;
            case 0x08: reason_str = "timeout"; break;
            case 0x05: reason_str = "auth failure"; break;
            case 0x06: reason_str = "key missing"; break;
            case 0x22: reason_str = "LL timeout"; break;
        }
        diag_trace_record(DIAG_EVT_GARMIN_DISCONNECT, reason, s_conn_handle, 0, 0, NULL, 0);
        rsc_log('W', "Garmin disconnected (0x%02X: %s)", reason, reason_str);
        s_connected = false;
        s_notifications_enabled = false;
        s_first_notify_logged = false;
        if (s_conn_cb) s_conn_cb(false);
        rsc_server_start_advertising();
        break;
    }

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_rsc_measurement_handle) {
            s_notifications_enabled = event->subscribe.cur_notify;
            diag_trace_record(DIAG_EVT_GARMIN_SUBSCRIBE, event->subscribe.cur_notify,
                              event->subscribe.attr_handle, event->subscribe.conn_handle, 0, NULL, 0);
            rsc_log('I', "Garmin %s RSC notifications",
                    event->subscribe.cur_notify ? "enabled" : "disabled");
        }
        break;

    case BLE_GAP_EVENT_ENC_CHANGE:
        diag_trace_record(DIAG_EVT_GARMIN_ENCRYPTION, event->enc_change.status, event->enc_change.conn_handle, 0, 0, NULL, 0);
        if (event->enc_change.status != 0) {
            rsc_log('E', "Encryption failed (status=%d)", event->enc_change.status);
        }
        break;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
            struct ble_sm_io pk = {0};
            pk.action = BLE_SM_IOACT_NUMCMP;
            pk.numcmp_accept = 1;
            ble_sm_inject_io(event->passkey.conn_handle, &pk);
        }
        break;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc desc;
        int rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        if (rc == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        rsc_diag_gap_callback(DIAG_EVT_GAP_CALLBACK_EXIT, event);
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_ADV_COMPLETE:
        rsc_server_start_advertising();
        break;

    default:
        break;
    }

    rsc_diag_gap_callback(DIAG_EVT_GAP_CALLBACK_EXIT, event);
    return 0;
}

esp_err_t rsc_server_init(rsc_conn_cb_t conn_cb, rsc_log_cb_t log_cb)
{
    s_conn_cb = conn_cb;
    s_log_cb = log_cb;
    s_connected = false;
    s_notifications_enabled = false;

    int rc = ble_gatts_count_cfg(rsc_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT count_cfg failed: %d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(rsc_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT add_svcs failed: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "RSC server initialized");
    return ESP_OK;
}

esp_err_t rsc_server_start_advertising(void)
{
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = 0x00A0,
        .itvl_max = 0x0190,
    };

    struct ble_hs_adv_fields adv_fields = {0};
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.uuids16 = (ble_uuid16_t[]){BLE_UUID16_INIT(0x1814)};
    adv_fields.num_uuids16 = 1;
    adv_fields.uuids16_is_complete = 1;
    adv_fields.appearance = 0x0441;
    adv_fields.appearance_is_present = 1;

    int rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Set adv fields failed: %d", rc);
        return ESP_FAIL;
    }

    struct ble_hs_adv_fields rsp_fields = {0};
    rsp_fields.name = (uint8_t *)ble_svc_gap_device_name();
    rsp_fields.name_len = strlen(ble_svc_gap_device_name());
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Set scan rsp fields failed: %d", rc);
        return ESP_FAIL;
    }

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                            &adv_params, rsc_gap_event, NULL);
    diag_trace_record(DIAG_EVT_ADV_RESULT, (uint32_t)rc, s_connected, s_notifications_enabled, 0, NULL, 0);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "Advertising start failed: %d", rc);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t rsc_server_notify(const rsc_data_t *data)
{
    if (!s_connected || !s_notifications_enabled) {
        diag_trace_record(DIAG_EVT_RSC_SKIPPED, s_connected, s_notifications_enabled,
                          data->speed_256ths_mps, data->total_distance_dm, NULL, 0);
        ESP_LOGD(TAG, "Notify skip: conn=%d notif=%d", s_connected, s_notifications_enabled);
        return ESP_OK;
    }

    uint8_t buf[10];
    uint8_t offset = 0;

    uint8_t flags = (1 << 1); // total distance present
    if (data->is_running) flags |= (1 << 2);
    buf[offset++] = flags;

    buf[offset++] = data->speed_256ths_mps & 0xFF;
    buf[offset++] = (data->speed_256ths_mps >> 8) & 0xFF;

    buf[offset++] = data->cadence_spm;

    buf[offset++] = data->total_distance_dm & 0xFF;
    buf[offset++] = (data->total_distance_dm >> 8) & 0xFF;
    buf[offset++] = (data->total_distance_dm >> 16) & 0xFF;
    buf[offset++] = (data->total_distance_dm >> 24) & 0xFF;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, offset);
    if (om == NULL) {
        ESP_LOGD(TAG, "mbuf alloc failed, skipping notify");
        return ESP_ERR_NO_MEM;
    }

    int rc = ble_gatts_notify_custom(s_conn_handle, s_rsc_measurement_handle, om);
    diag_trace_record(rc == 0 ? DIAG_EVT_RSC_SENT : DIAG_EVT_RSC_SKIPPED,
                      (uint32_t)rc, s_conn_handle, data->speed_256ths_mps,
                      data->total_distance_dm, buf, offset);
    if (rc == 0) {
        if (!s_first_notify_logged) {
            s_first_notify_logged = true;
            rsc_log('I', "RSC data flowing (spd=%d cad=%d)",
                    data->speed_256ths_mps, data->cadence_spm);
        }
    } else if (rc == BLE_HS_ENOMEM) {
        ESP_LOGD(TAG, "Notify backpressure, will retry next cycle");
    } else {
        rsc_log('E', "RSC notify failed: %d", rc);
    }

    return rc == 0 ? ESP_OK : ESP_FAIL;
}

bool rsc_server_is_connected(void)
{
    return s_connected;
}

bool rsc_server_notifications_enabled(void)
{
    return s_notifications_enabled;
}
