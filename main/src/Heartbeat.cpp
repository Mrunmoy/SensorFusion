#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "Heartbeat.hpp"

void Heartbeat::start()
{
    gpio_reset_pin(HEARTBEAT_GPIO);
    gpio_set_direction(HEARTBEAT_GPIO, GPIO_MODE_OUTPUT);

    xTaskCreatePinnedToCore(
        [](void*){
            while (true) {
                gpio_set_level(HEARTBEAT_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(HEARTBEAT_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        },
        "heartbeat_task",
        2048,
        nullptr,
        1,
        nullptr,
        tskNO_AFFINITY
    );
}
