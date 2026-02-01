#ifndef CONTROL_H
#define CONTROL_H

typedef struct {
    QueueHandle_t ibus_mailbox;
    QueueHandle_t state_estimate_mailbox;
} ControlConfig;

void control_task(void* arg);

void esc_home_task(void* arg);

#endif