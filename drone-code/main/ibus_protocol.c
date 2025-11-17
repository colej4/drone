#include <stdint.h> 

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "ibus_protocol.h"

#define UART_NUM        UART_NUM_2
#define UART_RX_PIN     GPIO_NUM_16
#define UART_TX_PIN     UART_PIN_NO_CHANGE
#define BUF_SIZE        256
#define IBUS_PKT_SIZE   32

static QueueHandle_t uart_queue;

uint16_t flip_endian_16(uint16_t value) {
    return (value >> 8) | (value << 8);
}

uint16_t bytes_to_uint16(uint8_t byte1, uint8_t byte2) {
    return (uint16_t)(byte1 | (byte2 << 8));
}

uint16_t calculate_checksum(const uint8_t *data, uint16_t length) {
    uint16_t sum = 0;
    for (uint16_t i = 0; i < length; i++) {
        sum += data[i];
    }
    uint16_t checksum = 0xFFFF - (uint16_t)sum;
    return checksum;
}

void ibus_uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // Install driver with RX buffer
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE, 0, 64, &uart_queue, 0));
    printf("IBUS UART initialized.");
}

void uart_event_task(void *arg) {
    uart_event_t event;
    uint8_t data[BUF_SIZE];

    uint8_t pkt[IBUS_PKT_SIZE];
    int pkt_index = 0;

    QueueHandle_t send_mailbox = *(QueueHandle_t*)arg;

    TickType_t last_print = 0;

    for (;;) {
        // Wait for UART events
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            switch (event.type) {

            case UART_DATA:
                // Read the incoming bytes into 'data'
                int len = uart_read_bytes(UART_NUM, data, event.size, 0);
                if (len != 32) {
                    printf("Warning: Received %d bytes from UART\n", len);
                }

                // Process each byte
                for (int i = 0; i < len; i++) {

                    pkt[pkt_index++] = data[i];

                    //full packet recieved
                    if (pkt_index >= IBUS_PKT_SIZE) {
                        pkt_index = 0;

                        // Check header
                        if (pkt[0] == 0x20 && pkt[1] == 0x40) {

                            uint16_t rx_cs = bytes_to_uint16(pkt[30], pkt[31]);
                            uint16_t calc_cs = calculate_checksum(pkt, 30);

                            if (rx_cs == calc_cs) {
                                TickType_t now = xTaskGetTickCount();
                                

                                uint16_t channels[6];
                                for (int ch = 0; ch <= 6; ch++) {
                                    uint16_t val =
                                        pkt[2 + ch*2] |
                                        (pkt[3 + ch*2] << 8);
                                    channels[ch] = val;
                                }
                                float roll = ((float)channels[0] - 1500.0f) / 500.0f;
                                float pitch = ((float)channels[1] - 1500.0f) / 500.0;
                                float throttle = ((float)channels[2] - 1000.0f) / 1000.0f;
                                float yaw = ((float)channels[3] - 1500.0f) / 500.0f;
                                float vra = ((float)channels[4] - 1000.0f) / 1000.0f;
                                float vrb = ((float)channels[5] - 1000.0f) / 1000.0f;

                                IbusMessage msg = {
                                    .throttle = throttle,
                                    .yaw = yaw,
                                    .pitch = pitch,
                                    .roll = roll,
                                    .vra = vra,
                                    .vrb = vrb
                                };

                                xQueueOverwrite(send_mailbox, &msg);
                            } else {
                                printf("Checksum error: received 0x%04X, calculated 0x%04X\n", rx_cs, calc_cs);
                            }
                        } else {
                            printf("Invalid IBUS packet header: 0x%02X 0x%02X\n", pkt[0], pkt[1]);
                            if (data[i-1] == 0x20 && data[i] == 0x40) {
                                printf("recovered start of packet at byte %d\n", i-1);
                                // Found start of IBUS packet
                                pkt_index = 2;
                                pkt[0] = 0x20;
                                pkt[1] = 0x40;
                            }
                        }
                    }
                }
                break;

            case UART_BUFFER_FULL:
                printf("UART buffer full\n");
                uart_flush_input(UART_NUM);
                break;
            case UART_FIFO_OVF:
                printf("UART FIFO overflow\n");
                uart_flush_input(UART_NUM);
                break;
            default:
                printf("Unhandled UART event type: %d\n", event.type);
                break;
            }
        }
    }
}