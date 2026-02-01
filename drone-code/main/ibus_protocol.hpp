#ifndef IBUS_PROTOCOL_H
#define IBUS_PROTOCOL_H


void ibus_task(void *arg);
void ibus_uart_init(void);


typedef struct {
    float throttle;
    float yaw;
    float pitch;
    float roll;
    float vra;
    float vrb;
} IbusMessage;

#endif


