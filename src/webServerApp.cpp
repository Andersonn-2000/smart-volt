#include "webServerApp.h"
#include "freertos/queue.h"

WebServer server(80);
extern QueueSetHandle_t xPowerDataQueue;
extern volatile int relayState;

void setupWebServer(){
    server.on("/", HTTP_GET, []() {server.send(200, "text/html", "<h1>ESP32 Monitor Iniciado</h1>"); });
    server.on("/status", handleStatus);
    server.begin();
}

void handleStatus(){
    PowerData_t data;
    xQueuePeek(xPowerDataQueue, &data, 0);

    String json = "{";
    json += "\"v\":" + String(data.voltageRMS, 1) + ",";
    json += "\"i\":" + String(data.currentRMS, 2) + ",";
    json += "\"p\":" + String(data.activePower, 1) + ",";
    json += "\"e\":" + String(data.energyTotal, 4) + ",";
    json += "\"relay\":" + String(relayState);
    json += "}";
    server.send(200, "application/json", json);
}

void vTaskServer(void* pvParameters){
    setupWebServer();
    while(true){
        server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}