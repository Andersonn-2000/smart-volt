#ifndef CONFIG_H
#define CONFIG_H
#include<Arduino.h>

// -- PINOUT --
#define RELAY_PIN 13
#define VOLTAGE_SENSOR_PIN 34
#define CURRENT_SENSOR_PIN 35

// -- NETWORK --
#define WIFI_SSID ""
#define WIFI_PASSWORD ""

// -- SENSORS --
#define ADC_MAX 4095.0
#define VREF 3.3
#define FREQUENCY 60
#define SAMPLES 1000

// -- SENSOR CALIBRATION --
#define VOLTAGE_SENSITIVITY 0.0125
#define ACS_SENSITIVITY 0.100
#define VOLTAGE_CAL 1.0
#define CURRENTE_CAL 1.0

// -- IDs WATCHDOG
#define TASK_ID_WIFI (1 << 0)
#define TASK_ID_SERVER (1 << 1)
#define TASK_ID_CONTROL (1 << 2)
#define TASK_ID_SENSORS (1 << 3)
#define TASK_ID_POWER (1 << 4)

#endif