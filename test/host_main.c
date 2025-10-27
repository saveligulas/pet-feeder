#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Forward declaration of app_main from your main component
void app_main(void);

// This is the main entry point when running on the host.
void host_main_task(void *arg) {
    ESP_LOGI("HOST_MAIN", "Host test application started.");
    app_main(); // Call your actual app_main
    vTaskDelete(NULL);
}

// The main function for the host build, which starts FreeRTOS.
int main(int argc, char **argv) {
    printf("Starting ESP-IDF Host Test...\n");
    xTaskCreate(host_main_task, "host_main", 4096, NULL, 5, NULL);
    vTaskStartScheduler();
    return 0;
}