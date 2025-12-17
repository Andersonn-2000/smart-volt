// Projeto: Controle de Lâmpada + Monitor de Tensão (ZMPT101B) + Corrente (ACS712)
// Placa: ESP32
// Autor: Adaptado para o usuário (Anderson)
// Data: 2025-12-12

// --- Bibliotecas ---
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h> // Biblioteca para criar o servidor HTTP no ESP32

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

// --- Pinout ---
#define RELAY_PIN 13            // Pino digital para controlar o módulo relé (lâmpada)
#define VOLTAGE_SENSOR_PIN 34   // Pino ADC para ler o sensor ZMPT101B (GPIO34 - ADC1_CH6)
#define CURRENT_SENSOR_PIN 35   // Pino ADC para ler o sensor ACS712 (GPIO35 - ADC1_CH7)

// --- Configurações de Rede ---
#define WIFI_SSID       "Xze"
#define WIFI_PASSPHRASE "280762004"

// Porta do servidor web
#define WEB_SERVER_PORT 80

// --- Configurações do Sensor ZMPT101B ---
#define SAMPLES 1000              // Número de amostras por leitura para tensão
#define ADC_MAX 4095.0            // Máximo valor do ADC (12-bit)
#define VREF 3.3                  // Tensão de referência do ESP32 (3.3V)
#define SENSITIVITY 0.0125        // Sensibilidade do ZMPT101B (V RMS per V sensor) - ajuste conforme necessário
#define VOLTAGE_CALIBRATION 1.0   // Fator de calibração para correção
#define FREQUENCY 60.0            // Frequência da rede (Hz) - Brasil: 60Hz

// --- Configurações do Sensor ACS712 ---
#define CURRENT_SAMPLES 1000      // número de amostras para corrente
#define ACS_SENSITIVITY 0.100     // V/A (ex.: 0.100 V/A para módulo 20A). Ajuste conforme seu módulo.
#define ACS_OFFSET (VREF / 2.0)   // Offset do sensor ACS712 (meio da Vref)
#define CURRENT_CALIBRATION 1.0   // fator de calibração da corrente

// --- Configurações de Tarefas (IDs para EventGroup) ---
#define TASK_ID_WIFI    (1 << 0)
#define TASK_ID_SERVER  (1 << 1)
#define TASK_ID_CONTROL (1 << 2)
#define TASK_ID_VOLTAGE (1 << 3)
#define TASK_ID_CURRENT (1 << 4)

#define ALL_TASKS_ID (TASK_ID_WIFI | TASK_ID_SERVER | TASK_ID_CONTROL | TASK_ID_VOLTAGE | TASK_ID_CURRENT)

// --- Recursos Compartilhados ---
// Objeto do Servidor Web
WebServer server(WEB_SERVER_PORT);

// Enum para o estado da lâmpada/relé
typedef enum {
    COMMAND_OFF,
    COMMAND_ON
} RelayCommand_t;

// Estrutura para dados de tensão
typedef struct {
    float voltageRMS;
    float voltagePeak;
    bool voltageOK;
} VoltageData_t;

// Estrutura para dados de corrente
typedef struct {
    float currentRMS;
    float currentPeak;
    bool currentOK;
} CurrentData_t;

// Filas para comunicação entre tarefas
QueueHandle_t xRelayCommandQueue;
QueueHandle_t xVoltageDataQueue;
QueueHandle_t xCurrentDataQueue;

// Semáforo Mutex para proteger o acesso à conexão Wi-Fi/HTTP
SemaphoreHandle_t xWifiMutex;

// Event Group para o Watchdog (Monitoramento de Tarefas)
EventGroupHandle_t xWatchdogGroupHandle;

// Variável compartilhada para estado do relé (protegida por mutex)
volatile int relayState = LOW;
SemaphoreHandle_t xRelayStateMutex;

// --- Conteúdo HTML da Interface (Servido pelo ESP32) ---
const char* HTML_CONTENT = R"raw(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Controle de Lâmpada Remoto - ESP32</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            display: flex;
            flex-direction: column;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
            margin: 0;
            background-color: #f4f4f9;
            text-align: center;
            padding: 20px;
        }
        .container {
            background-color: #ffffff;
            padding: 36px;
            border-radius: 12px;
            box-shadow: 0 6px 20px rgba(0, 0, 0, 0.08);
            max-width: 680px;
            width: 95%;
        }
        h1 {
            color: #333;
            margin-bottom: 20px;
        }
        .button-group {
            display: flex;
            gap: 12px;
            justify-content: center;
            margin-bottom: 20px;
            flex-wrap: wrap;
        }
        .control-button {
            text-decoration: none; 
            padding: 14px 22px;
            font-size: 1.1em;
            font-weight: bold;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            transition: all 0.18s ease;
            color: white;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.08);
        }
        .on-button { background-color: #4CAF50; }
        .on-button:hover { transform: translateY(-2px); }
        .off-button { background-color: #f44336; }
        .off-button:hover { transform: translateY(-2px); }
        .status-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 12px;
            margin-top: 12px;
        }
        .card {
            background-color: #f8f9fa;
            padding: 12px 14px;
            border-radius: 8px;
            text-align: left;
        }
        .label { font-weight: 700; color: #444; display:block; }
        .value { font-size: 1.25em; color: #222; margin-top:6px; }
        .voltage-ok { color: #4CAF50; font-weight: bold; }
        .voltage-danger { color: #f44336; font-weight: bold; }
        .current-ok { color: #4CAF50; font-weight:bold; }
        .current-warning { color: #ff9800; font-weight:bold; }
        .footer { color: #666; font-size: 0.85em; margin-top: 16px; }
        @media (max-width: 600px) {
            .status-grid { grid-template-columns: 1fr; }
            .control-button { width: 100%; }
        }
    </style>
    <script>
        function updateStatus() {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('voltage-rms').textContent = data.voltage_rms.toFixed(1) + ' V';
                    document.getElementById('voltage-peak').textContent = data.voltage_peak.toFixed(1) + ' V';
                    document.getElementById('relay-state').textContent = data.relay_state ? 'LIGADO' : 'DESLIGADO';
                    document.getElementById('relay-state').style.color = data.relay_state ? '#4CAF50' : '#f44336';

                    const vStatus = document.getElementById('voltage-status');
                    vStatus.textContent = data.voltage_ok ? 'NORMAL' : 'FORA DA FAIXA';
                    vStatus.className = data.voltage_ok ? 'voltage-ok' : 'voltage-danger';

                    document.getElementById('current-rms').textContent = data.current_rms.toFixed(2) + ' A';
                    document.getElementById('current-peak').textContent = data.current_peak.toFixed(2) + ' A';
                    const cStatus = document.getElementById('current-status');
                    cStatus.textContent = data.current_ok ? 'NORMAL' : 'ALERTA';
                    cStatus.className = data.current_ok ? 'current-ok' : 'current-warning';

                    document.getElementById('last-update').textContent = data.timestamp;
                })
                .catch(error => {
                    console.error('Erro ao atualizar status:', error);
                });
        }
        setInterval(updateStatus, 2000);
        document.addEventListener('DOMContentLoaded', updateStatus);
    </script>
</head>
<body>
    <div class="container">
        <h1>💡 Controle de Lâmpada + Monitor de Tensão & Corrente</h1>
        <div class="button-group">
            <a href="/on" class="control-button on-button">LIGAR LÂMPADA</a>
            <a href="/off" class="control-button off-button">DESLIGAR LÂMPADA</a>
        </div>

        <div class="status-grid">
            <div class="card">
                <span class="label">Tensão RMS</span>
                <span class="value" id="voltage-rms">-- V</span>
                <span class="label" style="margin-top:8px">Tensão de Pico</span>
                <span class="value" id="voltage-peak">-- V</span>
                <span class="label" style="margin-top:8px">Status da Tensão</span>
                <span class="value" id="voltage-status">--</span>
            </div>

            <div class="card">
                <span class="label">Corrente RMS</span>
                <span class="value" id="current-rms">-- A</span>
                <span class="label" style="margin-top:8px">Corrente de Pico</span>
                <span class="value" id="current-peak">-- A</span>
                <span class="label" style="margin-top:8px">Status da Corrente</span>
                <span class="value" id="current-status">--</span>
            </div>

            <div class="card">
                <span class="label">Estado do Relé</span>
                <span class="value" id="relay-state">--</span>
                <span class="label" style="margin-top:8px">Última Atualização</span>
                <span class="value" id="last-update">--</span>
            </div>

            <div class="card">
                <span class="label">Informações</span>
                <span class="value">Atualização automática a cada 2 segundos</span>
            </div>
        </div>

        <p class="footer">ESP32 · ZMPT101B (tensão) · ACS712 (corrente)</p>
    </div>
</body>
</html>
)raw";

// --- Funções Auxiliares para Leitura de Tensão e Corrente ---

/**
 * @brief Lê a tensão RMS usando o sensor ZMPT101B
 * @return Estrutura VoltageData_t com os dados medidos
 */
VoltageData_t readVoltage() {
    VoltageData_t data;
    float sumSq = 0.0f;
    float maxVal = -10.0f;
    float minVal = 10.0f;

    // tempo total do período (1 ciclo) em micros / amostras
    float sampleIntervalMicros = (1000000.0f / FREQUENCY) / (float)SAMPLES;

    for (int i = 0; i < SAMPLES; i++) {
        int adc = analogRead(VOLTAGE_SENSOR_PIN);
        float sensorVoltage = ((float)adc / ADC_MAX) * VREF; // 0..Vref

        // remover offset (meio da faixa)
        float centered = sensorVoltage - (VREF / 2.0f);

        if (centered > maxVal) maxVal = centered;
        if (centered < minVal) minVal = centered;

        sumSq += centered * centered;

        // espaçar amostras para cobrir 1 ciclo
        delayMicroseconds((int)sampleIntervalMicros);
    }

    // Calcula Vpp, Vpeak e Vrms do sinal no lado do sensor
    float vpp_sensor = maxVal - minVal;
    float vpeak_sensor = vpp_sensor / 2.0f;              // pico (sensor volts)
    float vrms_sensor = vpeak_sensor / sqrtf(2.0f);      // Vrms (sensor volts)

    // Converte para tensão de rede usando sensibilidade e calibração
    // Dependendo do módulo, SENSITIVITY pode representar Vsensor_per_Vac RMS
    float measuredVrms = (vrms_sensor / SENSITIVITY) * VOLTAGE_CALIBRATION;
    float measuredVpeak = (vpeak_sensor / SENSITIVITY) * VOLTAGE_CALIBRATION;

    data.voltageRMS = measuredVrms;
    data.voltagePeak = measuredVpeak;

    // Verifica se a tensão está em faixa segura (ajuste conforme sua rede)
    data.voltageOK = (data.voltageRMS > 90.0f && data.voltageRMS < 130.0f) ||
                     (data.voltageRMS > 198.0f && data.voltageRMS < 242.0f);

    return data;
}

/**
 * @brief Lê a corrente RMS usando o sensor ACS712
 * @return Estrutura CurrentData_t com os dados medidos
 */
CurrentData_t readCurrent() {
    CurrentData_t data;
    float sumSq = 0.0f;
    float maxVal = -10.0f;
    float minVal = 10.0f;

    float sampleIntervalMicros = (1000000.0f / FREQUENCY) / (float)CURRENT_SAMPLES;

    for (int i = 0; i < CURRENT_SAMPLES; i++) {
        int adc = analogRead(CURRENT_SENSOR_PIN);
        float sensorVoltage = ((float)adc / ADC_MAX) * VREF; // 0..Vref

        // Remove offset (ACS outputs Vcc/2 at 0A)
        float centered = sensorVoltage - ACS_OFFSET;

        if (centered > maxVal) maxVal = centered;
        if (centered < minVal) minVal = centered;

        sumSq += centered * centered;

        delayMicroseconds((int)sampleIntervalMicros);
    }

    // Vpp, Vpeak, Vrms on sensor side
    float vpp_sensor = maxVal - minVal;
    float vpeak_sensor = vpp_sensor / 2.0f;              // pico (sensor volts)
    float vrms_sensor = vpeak_sensor / sqrtf(2.0f);      // Vrms (sensor volts)

    // Converte sensor volts para corrente usando sensibilidade (V per A)
    float measuredIrms = (vrms_sensor / ACS_SENSITIVITY) * CURRENT_CALIBRATION;
    float measuredIpeak = (vpeak_sensor / ACS_SENSITIVITY) * CURRENT_CALIBRATION;

    data.currentRMS = measuredIrms;
    data.currentPeak = measuredIpeak;

    // Faixa de segurança: por padrão assume que < 10A é OK (ajuste conforme carga)
    data.currentOK = (data.currentRMS < 10.0f);

    return data;
}

// --- Definição das Funções de Manipulação de Requisição ---

void handleRoot() {
    // Envia a página HTML completa para o cliente
    server.send(200, "text/html", HTML_CONTENT);
}

void handleOn() {
    RelayCommand_t command = COMMAND_ON;
    xQueueSend(xRelayCommandQueue, &command, 0);

    // Redireciona para /
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "Ligando a lâmpada");
    Serial.println("Comando ON recebido e enviado para fila.");
}

void handleOff() {
    RelayCommand_t command = COMMAND_OFF;
    xQueueSend(xRelayCommandQueue, &command, 0);

    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "Desligando a lâmpada");
    Serial.println("Comando OFF recebido e enviado para fila.");
}

void handleStatus() {
    // Obtém dados de tensão mais recentes (não bloquear)
    VoltageData_t voltageData;
    CurrentData_t currentData;

    if (xQueueReceive(xVoltageDataQueue, &voltageData, 0) != pdTRUE) {
        voltageData.voltageRMS = 0.0f;
        voltageData.voltagePeak = 0.0f;
        voltageData.voltageOK = false;
    }

    if (xQueueReceive(xCurrentDataQueue, &currentData, 0) != pdTRUE) {
        currentData.currentRMS = 0.0f;
        currentData.currentPeak = 0.0f;
        currentData.currentOK = false;
    }

    // Obtém estado atual do relé
    int currentRelayState;
    xSemaphoreTake(xRelayStateMutex, portMAX_DELAY);
    currentRelayState = relayState;
    xSemaphoreGive(xRelayStateMutex);

    // Cria resposta JSON
    String json = "{";
    json += "\"voltage_rms\":" + String(voltageData.voltageRMS, 2) + ",";
    json += "\"voltage_peak\":" + String(voltageData.voltagePeak, 2) + ",";
    json += "\"voltage_ok\":" + String(voltageData.voltageOK ? "true" : "false") + ",";

    json += "\"current_rms\":" + String(currentData.currentRMS, 3) + ",";
    json += "\"current_peak\":" + String(currentData.currentPeak, 3) + ",";
    json += "\"current_ok\":" + String(currentData.currentOK ? "true" : "false") + ",";

    json += "\"relay_state\":" + String(currentRelayState == HIGH ? "true" : "false") + ",";
    json += "\"timestamp\":\"" + String(millis() / 1000) + "s\"";
    json += "}";

    server.send(200, "application/json", json);
}

// --- Definição das Tarefas FreeRTOS ---

void vTaskWifiConnect(void* pvParameters) {
    xSemaphoreTake(xWifiMutex, portMAX_DELAY);

    Serial.println("Conectando ao WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
        // opcional: timeout se quiser
        // if (millis() - start > 20000) break;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi conectado! IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nFalha ao conectar WiFi.");
    }

    xSemaphoreGive(xWifiMutex);
    xEventGroupSetBits(xWatchdogGroupHandle, TASK_ID_WIFI);

    vTaskDelete(NULL);
}

void vTaskServer(void* pvParameters) {
    // Espera WiFi conectar (watchdog garante sinal)
    xEventGroupWaitBits(xWatchdogGroupHandle, TASK_ID_WIFI, pdFALSE, pdTRUE, portMAX_DELAY);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/on", HTTP_GET, handleOn);
    server.on("/off", HTTP_GET, handleOff);
    server.on("/status", HTTP_GET, handleStatus);

    server.begin();
    Serial.println("Servidor HTTP iniciado.");

    while (true) {
        server.handleClient();
        xEventGroupSetBits(xWatchdogGroupHandle, TASK_ID_SERVER);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vTaskRelayControl(void* pvParameters) {
    RelayCommand_t command;
    int currentState = LOW;

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, currentState);

    // Atualiza variável compartilhada
    xSemaphoreTake(xRelayStateMutex, portMAX_DELAY);
    relayState = currentState;
    xSemaphoreGive(xRelayStateMutex);

    Serial.printf("Relé (Pin %d) inicializado em LOW (OFF)\n", RELAY_PIN);

    while (true) {
        if (xQueueReceive(xRelayCommandQueue, &command, portMAX_DELAY) == pdTRUE) {
            int newState = (command == COMMAND_ON) ? HIGH : LOW;

            if (newState != currentState) {
                digitalWrite(RELAY_PIN, newState);
                currentState = newState;

                // Atualiza variável compartilhada
                xSemaphoreTake(xRelayStateMutex, portMAX_DELAY);
                relayState = currentState;
                xSemaphoreGive(xRelayStateMutex);

                Serial.printf("-> NOVO COMANDO EXECUTADO: Relé alterado para %s\n",
                             (currentState == HIGH ? "ON" : "OFF"));
            } else {
                Serial.println("Comando recebido igual ao estado atual (sem mudança).");
            }
        }

        xEventGroupSetBits(xWatchdogGroupHandle, TASK_ID_CONTROL);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vTaskVoltageMonitoring(void* pvParameters) {
    VoltageData_t voltageData;

    // Configuração do pino ADC
    analogReadResolution(12); // 12-bit resolution (0-4095)
    analogSetPinAttenuation(VOLTAGE_SENSOR_PIN, ADC_11db);
    // note: analogSetAttenuation is deprecated in some cores; using per-pin when available

    Serial.println("Tarefa de monitoramento de tensão iniciada.");

    while (true) {
        voltageData = readVoltage();

        // Envia dados para a fila (overwrite para armazenar apenas o último)
        xQueueOverwrite(xVoltageDataQueue, &voltageData);

        static unsigned long lastLog = 0;
        if (millis() - lastLog > 5000) {
            Serial.printf("Tensão RMS: %.1fV, Pico: %.1fV, Status: %s\n",
                         voltageData.voltageRMS,
                         voltageData.voltagePeak,
                         voltageData.voltageOK ? "OK" : "FORA DA FAIXA");
            lastLog = millis();
        }

        xEventGroupSetBits(xWatchdogGroupHandle, TASK_ID_VOLTAGE);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Atualiza a cada segundo
    }
}

void vTaskCurrentMonitoring(void* pvParameters) {
    CurrentData_t currentData;

    // Configuração do pino ADC
    analogReadResolution(12);
    analogSetPinAttenuation(CURRENT_SENSOR_PIN, ADC_11db);

    Serial.println("Tarefa de monitoramento de corrente iniciada.");

    while (true) {
        currentData = readCurrent();

        xQueueOverwrite(xCurrentDataQueue, &currentData);

        static unsigned long lastLog = 0;
        if (millis() - lastLog > 5000) {
            Serial.printf("Corrente RMS: %.3fA, Pico: %.3fA, Status: %s\n",
                          currentData.currentRMS,
                          currentData.currentPeak,
                          currentData.currentOK ? "OK" : "ALTA");
            lastLog = millis();
        }

        xEventGroupSetBits(xWatchdogGroupHandle, TASK_ID_CURRENT);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Atualiza a cada segundo
    }
}

void vTaskWatchdog(void* pvParameters) {
    const TickType_t waitTicks = pdMS_TO_TICKS(15000);

    while (true) {
        // Aguarda que todas as flags sejam definidas nos últimos waitTicks
        EventBits_t status = xEventGroupWaitBits(
            xWatchdogGroupHandle,
            ALL_TASKS_ID,
            pdTRUE,   // limpar bits após ler
            pdTRUE,   // aguardar todos os bits
            waitTicks
        );

        if ((status & ALL_TASKS_ID) == ALL_TASKS_ID) {
            // Tudo ok — pode logar se quiser (comentado)
            // Serial.println("Watchdog: todas as tarefas reportaram atividade.");
        } else {
            Serial.println("--- WATCHDOG ALERTA: Falha em alguma tarefa! ---");
            Serial.printf("Status: 0x%08X\n", (unsigned int)status);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// --- Setup e Loop ---

void setup() {
    Serial.begin(115200);
    delay(100);

    // Inicializa recursos FreeRTOS
    xRelayCommandQueue = xQueueCreate(5, sizeof(RelayCommand_t));
    xVoltageDataQueue = xQueueCreate(1, sizeof(VoltageData_t));
    xCurrentDataQueue = xQueueCreate(1, sizeof(CurrentData_t));
    xWifiMutex = xSemaphoreCreateMutex();
    xRelayStateMutex = xSemaphoreCreateMutex();
    xWatchdogGroupHandle = xEventGroupCreate();

    // Inicializa pinos
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
    relayState = LOW;

    // Cria as tarefas
    xTaskCreate(vTaskWifiConnect, "Wifi Connect", 4096, NULL, 3, NULL);
    xTaskCreate(vTaskRelayControl, "Relay Control", 4096, NULL, 2, NULL);
    xTaskCreate(vTaskVoltageMonitoring, "Voltage Monitor", 8192, NULL, 2, NULL);
    xTaskCreate(vTaskCurrentMonitoring, "Current Monitor", 8192, NULL, 2, NULL);
    xTaskCreate(vTaskServer, "Web Server", 8192, NULL, 1, NULL);
    xTaskCreate(vTaskWatchdog, "Task Watchdog", 4096, NULL, 1, NULL);

    Serial.println("Sistema inicializado. Aguardando conexão WiFi...");
}

void loop() {
    // Não usamos loop(); deleta a tarefa principal por segurança
    vTaskDelete(NULL);
}
