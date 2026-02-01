#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#define LOG_LINE_MAX     256
#define LOG_QUEUE_LEN    128

#define UDP_BATCH_MAX    2000

#define FLUSH_PERIOD_MS  20

typedef struct {
    uint16_t n;
    char line[LOG_LINE_MAX];
} log_item_t;

static QueueHandle_t s_log_q;
static vprintf_like_t s_prev_vprintf;

static int s_sock = -1;
static struct sockaddr_in s_dst;

static volatile uint32_t s_dropped = 0;
static volatile uint32_t s_sent_pkts = 0;
static volatile uint32_t s_sent_bytes = 0;

static int log_capture_vprintf(const char *fmt, va_list ap)
{
    log_item_t item;

    int n = vsnprintf(item.line, sizeof(item.line), fmt, ap);

    if (n < 0) n = 0;
    if (n >= (int)sizeof(item.line)) n = (int)sizeof(item.line) - 1;
    item.n = (uint16_t)n;

    if (s_log_q) {
        if (xQueueSend(s_log_q, &item, 0) != pdTRUE) {
            s_dropped++;
        }
    }

    return n;
}

static void udp_flush(const uint8_t *buf, size_t len)
{
    if (len == 0) return;
    if (s_sock < 0) return;

    int rc = sendto(s_sock, buf, len, 0, (struct sockaddr *)&s_dst, sizeof(s_dst));
    if (rc > 0) {
        s_sent_pkts++;
        s_sent_bytes += (uint32_t)rc;
    }
}

static void log_sender_task(void *arg)
{
    (void)arg;

    uint8_t out[UDP_BATCH_MAX];
    size_t out_len = 0;

    TickType_t last_flush = xTaskGetTickCount();
    const TickType_t flush_period_ticks = pdMS_TO_TICKS(FLUSH_PERIOD_MS);

    for (;;) {
        log_item_t item;

        TickType_t now = xTaskGetTickCount();
        TickType_t elapsed = now - last_flush;
        TickType_t wait = (elapsed >= flush_period_ticks) ? 0 : (flush_period_ticks - elapsed);

        if (xQueueReceive(s_log_q, &item, wait) == pdTRUE) {

            // Flush if this line would exceed batch buffer
            if (out_len + item.n > UDP_BATCH_MAX) {
                udp_flush(out, out_len);
                out_len = 0;
                last_flush = xTaskGetTickCount();
            }

            memcpy(out + out_len, item.line, item.n);
            out_len += item.n;

            // Flush early if nearly full to avoid fragmentation
            if (out_len >= (UDP_BATCH_MAX - 64)) {
                udp_flush(out, out_len);
                out_len = 0;
                last_flush = xTaskGetTickCount();
            }
        }

        // Time-based flush
        now = xTaskGetTickCount();
        if ((now - last_flush) >= flush_period_ticks) {
            udp_flush(out, out_len);
            out_len = 0;
            last_flush = now;
        }
    }
}


void log_stream_init_udp(const char *laptop_ip, uint16_t port)
{
    s_log_q = xQueueCreate(LOG_QUEUE_LEN, sizeof(log_item_t));

    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    memset(&s_dst, 0, sizeof(s_dst));
    s_dst.sin_family = AF_INET;
    s_dst.sin_port = htons(port);
    inet_aton(laptop_ip, &s_dst.sin_addr);

    s_prev_vprintf = esp_log_set_vprintf(log_capture_vprintf);

    xTaskCreatePinnedToCore(log_sender_task, "log_sender", 4096, NULL, 5, NULL, 0);
}

void log_stream_get_stats(uint32_t *dropped, uint32_t *pkts, uint32_t *bytes)
{
    if (dropped) *dropped = s_dropped;
    if (pkts)    *pkts = s_sent_pkts;
    if (bytes)   *bytes = s_sent_bytes;
}
