set(EMBEDDED_HTML assets/index.html)

idf_component_register(
    SRCS
        "main.cpp"
        "src/WifiManager.cpp"
        "src/WebSocketServer.cpp"
        "src/Heartbeat.cpp"
        "src/SensorAdcBus.cpp"
        "src/Ad8232Driver.cpp"
        "src/Tasks.cpp"
    INCLUDE_DIRS
        "inc"
        ${CMAKE_CURRENT_LIST_DIR}/../third_party/regbus/include
    PRIV_REQUIRES
        esp_wifi
        esp_driver_i2c
        freertos
        esp_timer
        nvs_flash
        driver
        esp_event
        esp_netif
        fatfs
        spiffs
        esp_http_server
        esp_adc
    EMBED_FILES index.html
)
