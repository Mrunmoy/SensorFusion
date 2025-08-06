#pragma once
#include "driver/gpio.h"

#define HEARTBEAT_GPIO GPIO_NUM_12

class Heartbeat {
public:
    static void start();
};
