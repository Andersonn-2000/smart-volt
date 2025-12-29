#include "sensors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

extern QueueHandle_t xPowerDataQueue;
extern EventGroupHandle_t xWatchDogGroupHandle;

static float accumulatedEnergy = 0; // em Wh

void initSensors(){
    analogReadResolution(12);
    analogSetPinAttenuation(VOLTAGE_SENSOR_PIN, ADC_11db);
    analogSetPinAttenuation(CURRENT_SENSOR_PIN, ADC_11db);
}

void vTaskSensorReading(void* pvParameters){
    while(true){
        float sumVsq = 0, sumIsq = 0;
        int count = 0;

        unsigned long startTime = micros();
        
        // coleta amostras por 2 ciclos
        while(micros() - startTime < 33333){
            float vRaw = (analogRead(VOLTAGE_SENSOR_PIN) * (VREF/ADC_MAX)) - (VREF/2.0);
            float iRaw = (analogRead(CURRENT_SENSOR_PIN) * (VREF/ADC_MAX)) - (VREF/2.0);

            sumVsq += vRaw * vRaw;
            sumIsq += iRaw * iRaw;
            count++;
            delayMicroseconds(100);
        }

        PowerData_t measurement;
        measurement.voltageRMS = (sqrt(sumVsq/count) / VOLTAGE_SENSITIVITY) * VOLTAGE_CAL;
        measurement.currentRMS = (sqrt(sumIsq/count) / ACS_SENSITIVITY) * CURRENTE_CAL;

        // filtro de ruído para corrente baixa
        
    }
}