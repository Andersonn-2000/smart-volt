#include<Arduino.h>
#include<WiFi.h>
#include "config.h"
#include "sensors.h"
#include "webServerApp.h"

// GLOBAL RESOURCES
QueueHandle_t xPowerDataQueue;
QueueHandle_t xRelayCommandQueue;
EventGroupHandle_t xWatchDogGroupHandle;
volatile int relayState = LOW;

void setup(){
    Serial.begin(115200);

    xPowerDataQueue = xQueueCreate(1, sizeof(PowerData_t));
    xWatchDogGroupHandle = xEventGroupCreate();

    initSensors();
    pinMode(RELAY_PIN, OUTPUT);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // TASK CREATION
    xTaskCreate(vTaskSensorReading, "Sensors", 4096, NULL, 3, NULL);
    xTaskCreate(vTaskPowerCalculation, "Power", 4096, NULL, 2, NULL);
    xTaskCreate(vtaskServer, "Web", 8192, NULL, 1, NULL);

    Serial.println("Sistema Modularizado...");
}

void loop(){
    
}