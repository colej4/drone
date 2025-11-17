# Custom Drone Flight Controller Code

Built using freeRTOS in C with an ESP32.

## ibus_protocol.c

ibus_protocol.c contains a task which reads 6-channel messages over UART using the IBUS protocol. Meant for use with the FS-IA6B I have currently.