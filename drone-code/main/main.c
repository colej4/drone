#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"

#include "ibus_protocol.h"


void app_main(void)
{
    //initialize IBUS UART (for listening from controller)
    ibus_uart_init();
    //create mailbox for recieving IBUS messages
    QueueHandle_t ibus_mailbox = xQueueCreate(1, sizeof(IbusMessage));

    xTaskCreate(uart_event_task, "ibus_task", 2048, &ibus_mailbox, 1, NULL);

    while(1) {
        IbusMessage msg;
        if (xQueuePeek(ibus_mailbox, &msg, pdMS_TO_TICKS(100))) {
            // Process received IBUS message
            printf("Received IBUS Message - Throttle: %.2f, Yaw: %.2f, Pitch: %.2f, Roll: %.2f, VRA: %.2f, VRB: %.2f\n",
                   msg.throttle, msg.yaw, msg.pitch, msg.roll, msg.vra, msg.vrb);
            vTaskDelay(pdMS_TO_TICKS(100)); // Simulate processing delay
        } else {
            printf("failed to peek mailbox\n");
        }
    }
}