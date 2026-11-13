#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    DIAG_EVT_FTMS_RX = 1,
    DIAG_EVT_RSC_READY = 2,
    DIAG_EVT_RSC_SENT = 3,
    DIAG_EVT_RSC_SKIPPED = 4,
    DIAG_EVT_GARMIN_CONNECT = 5,
    DIAG_EVT_GARMIN_DISCONNECT = 6,
    DIAG_EVT_GARMIN_SUBSCRIBE = 7,
    DIAG_EVT_GARMIN_ENCRYPTION = 8,
    DIAG_EVT_ADV_RESULT = 9,
    DIAG_EVT_FTMS_SUBSCRIBE = 10,
    DIAG_EVT_FTMS_DISCONNECT = 11,
    DIAG_EVT_GAP_CALLBACK_ENTER = 12,
    DIAG_EVT_GAP_CALLBACK_EXIT = 13
} diag_event_type_t;

typedef struct {
    int64_t time_us;
    uint32_t sequence;
    uint8_t type;
    uint8_t data_len;
    uint8_t data[8];
    uint32_t a;
    uint32_t b;
    uint32_t c;
    uint32_t d;
} diag_event_t;

void diag_trace_init(void);
void diag_trace_record(uint8_t type, uint32_t a, uint32_t b,
                       uint32_t c, uint32_t d,
                       const uint8_t *data, uint8_t data_len);
size_t diag_trace_copy(diag_event_t *out, size_t capacity);
uint32_t diag_trace_total(void);
