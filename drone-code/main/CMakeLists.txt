idf_component_register(
    SRCS "wifi_logger.cpp" "wifi.cpp" "control.cpp" "state_estimator.cpp" "imu.cpp" "math_helpers.cpp" "feedforward.cpp" "pid.cpp" "ibus_protocol.cpp" "main.cpp" "ekf.cpp" "ekf_estimator.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES eigen driver esp_wifi nvs_flash
)
