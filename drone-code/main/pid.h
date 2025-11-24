typedef struct {
    float kP;
    float kI;
    float kD;
    float integral;
    float integral_bound;
    float derivative_ema_gain;
    float filtered_derivative;
    float previous_error;
    uint64_t previous_timestamp; //timestamp in micros
    float setpoint;
} PIDController;

typedef struct {
    gptimer_handle_t timer;
    QueueHandle_t input_queue;   // Queue for setpoint and measurement
    QueueHandle_t output_mailbox;  // Mailbox for control output
    float kP;
    float kI;
    float kD;
    char unique_id;
    float integral_bound;
    float derivative_ema_gain; 
    float initial_setpoint;
} PIDConfig;

enum PIDMessageType{
    MEASUREMENT,
    SETPOINT
};

typedef struct {
    float data;
    enum PIDMessageType type;
} PIDMessage;

PIDController* new_pid(float kP, float kI, float kD, float integral_bound, float derivative_ema_gain, float initial_setpoint, uint64_t timestamp);

float calculate_pid(PIDController* controller, float measurement, uint64_t timestamp);

void vPID_controller(void *pvParameters);