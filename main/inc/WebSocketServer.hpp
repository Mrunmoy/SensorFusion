#pragma once

#include "esp_http_server.h"

class WebSocketServer {
public:
    static void start();
    static void broadcast(const char* message);
};
