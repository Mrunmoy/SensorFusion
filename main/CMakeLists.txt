set(EMBEDDED_HTML assets/index.html)

idf_component_register(
    SRCS
        "main.cpp"
        "src/WifiManager.cpp"
        "src/WebSocketServer.cpp"
        "src/Heartbeat.cpp"
        "src/SensorFusionSim.cpp"
        "src/SensorI2CBus.cpp"
        "src/MPU6050Driver.cpp"
        "src/ADXL345Driver.cpp"
        "src/QMC5883LDriver.cpp"
        "src/BMP180Driver.cpp"
    INCLUDE_DIRS
        "inc"
    PRIV_REQUIRES
        esp_wifi
        nvs_flash
        driver
        esp_event
        esp_netif
        fatfs
        spiffs
        esp_http_server
    EMBED_FILES index.html
)
