#ifndef WEBSERVERAPP_h
#define WEBSERVERAPP_H

#include <WebServer.h>
#include "sensors.h"

void setupWebServer();
void handleRoot();
void handleStatus();
void vtaskServer(void* pvParameters);

#endif