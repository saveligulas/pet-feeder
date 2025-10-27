
if(TARGET_ARCH STREQUAL "host")

    idf_component_register(
        SRCS "host_main.c"
        REQUIRES freertos esp_system
    )

    idf_component_register(
        NAME garag__esp-idf-pn532
        SRCS "mock_pn532_driver.c"
        INCLUDE_DIRS "include"
    )

endif()