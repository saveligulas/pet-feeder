idf_component_register(
    SRCS "src/pi_communicator.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_client esp_wifi nvs_flash rfid_reader espressif__cjson
)
