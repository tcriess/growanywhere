
#include <Arduino.h>

#define TINY_GSM_MODEM_SIM7080

#define SerialMon           Serial
#define MONITOR_BAUDRATE    115200

#define SerialAT            Serial1
#define SIM7080_BAUDRATE    115200

#define IoT_BASE_SIM7080_RESET       -1
#define IoT_BASE_SIM7080_EN          12
#define IoT_BASE_SIM7080_TX          0
#define IoT_BASE_SIM7080_RX          35

#define IoT_BASE_RS485_TX          15
#define IoT_BASE_RS485_RX          13

void iotBaseInit() {
    pinMode(IoT_BASE_SIM7080_EN, OUTPUT);
    digitalWrite(IoT_BASE_SIM7080_EN, LOW);
    delay(2000);
    digitalWrite(IoT_BASE_SIM7080_EN, HIGH);
    delay(500);
};