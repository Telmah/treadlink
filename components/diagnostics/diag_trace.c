#include "diag_trace.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <string.h>

#define DIAG_RING_SIZE 2048

static diag_event_t s_ring[DIAG_RING_SIZE];
static uint32_t s_write_index;
static uint32_t s_sequence;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void diag_trace_init(void)
{
    portENTER_CRITICAL(&s_mux);
    memset(s_ring, 0, sizeof(s_ring));
    s_write_index = 0;
    s_sequence = 0;
    portEXIT_CRITICAL(&s_mux);
}

void diag_trace_record(uint8_t type, uint32_t a, uint32_t b,
                       uint32_t c, uint32_t d,
                       const uint8_t *data, uint8_t data_len)
{
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_mux);
    diag_event_t *e = &s_ring[s_write_index % DIAG_RING_SIZE];
    e->time_us = now;
    e->sequence = ++s_sequence;
    e->type = type;
    e->a = a; e->b = b; e->c = c; e->d = d;
    e->data_len = data_len > sizeof(e->data) ? sizeof(e->data) : data_len;
    memset(e->data, 0, sizeof(e->data));
    if (data && e->data_len) memcpy(e->data, data, e->data_len);
    s_write_index++;
    portEXIT_CRITICAL(&s_mux);
}

size_t diag_trace_copy(diag_event_t *out, size_t capacity)
{
    if (!out || !capacity) return 0;
    portENTER_CRITICAL(&s_mux);
    uint32_t available = s_write_index < DIAG_RING_SIZE ? s_write_index : DIAG_RING_SIZE;
    size_t count = available < capacity ? available : capacity;
    uint32_t first = s_write_index - (uint32_t)count;
    for (size_t i = 0; i < count; i++) out[i] = s_ring[(first + (uint32_t)i) % DIAG_RING_SIZE];
    portEXIT_CRITICAL(&s_mux);
    return count;
}

size_t diag_trace_copy_range(diag_event_t *out, uint32_t first_sequence,
                             uint32_t last_sequence, size_t capacity)
{
    if (!out || !capacity || first_sequence == 0 || last_sequence == 0) return 0;

    portENTER_CRITICAL(&s_mux);

    uint32_t oldest = s_sequence > DIAG_RING_SIZE
                    ? s_sequence - DIAG_RING_SIZE + 1U : 1U;
    if (first_sequence < oldest) first_sequence = oldest;
    if (last_sequence > s_sequence) last_sequence = s_sequence;

    size_t count = 0;
    if (first_sequence <= last_sequence) {
        uint32_t available = last_sequence - first_sequence + 1U;
        count = available < capacity ? available : capacity;

        for (size_t i = 0; i < count; i++) {
            uint32_t sequence = first_sequence + (uint32_t)i;
            const diag_event_t *event = &s_ring[(sequence - 1U) % DIAG_RING_SIZE];
            if (event->sequence != sequence) {
                count = i;
                break;
            }
            out[i] = *event;
        }
    }

    portEXIT_CRITICAL(&s_mux);
    return count;
}

uint32_t diag_trace_total(void)
{
    portENTER_CRITICAL(&s_mux);
    uint32_t total = s_sequence;
    portEXIT_CRITICAL(&s_mux);
    return total;
}
