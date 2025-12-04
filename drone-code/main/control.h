typedef struct {
    QueueHandle_t ibus_mailbox;
    QueueHandle_t state_estimate_mailbox;
} ControlConfig;