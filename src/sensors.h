#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"

struct PowerData_t{
    float voltageRMS;
    float currentRMS;
    float activePower; // Watts
    float energyTotal; // kWh
    bool statusOK;
};

void initSensors();
void vTaskSensorReading(void* pvParameters);
void vTaskPowerCalculation(void* pvParameters);

#endif