void state_estimator_task(void *arg);

typedef struct {
    QueueHandle_t imu_data_queue;
    QueueHandle_t state_estimate_mailbox;
} StateEstimatorConfig;