#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"

#include "rfid.h"
#include "pi_communicator.h"

static const char *TAG = "APP_MAIN";

#define RFID_POLL_PERIOD_MS    500
#define PI_COMM_TASK_STACK_SIZE 4096
#define RFID_POLL_TASK_STACK_SIZE 3072
#define RFID_UID_QUEUE_LENGTH  10
#define PI_ENDPOINT_URL        "http://192.168.1.100:5000/rfid_data" // CRITICAL: Replace with your RPi's actual IP and endpoint

static QueueHandle_t rfid_uid_queue;

static void rfid_uid_detected_callback(const RFID_Uid_t* uid) {
    if (uid == NULL) return;

    ESP_LOGI(TAG, "RFID UID detected");

    RFID_Uid_t* uid_copy = (RFID_Uid_t*)malloc(sizeof(RFID_Uid_t) + uid->uid_len);
    if (uid_copy == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for UID copy.");
        return;
    }
    uid_copy->uid_len = uid->uid_len;
    memcpy(uid_copy->uid, uid->uid, uid->uid_len);

    if (xQueueSend(rfid_uid_queue, &uid_copy, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGW(TAG, "RFID UID queue full, dropping UID.");
        free(uid_copy);
    }
}

static void pi_communicator_task(void* pvParameters) {
    ESP_LOGI(TAG, "PI Communicator task started.");
    RFID_Uid_t* received_uid;

    while (1) {
        if (xQueueReceive(rfid_uid_queue, &received_uid, portMAX_DELAY) == pdPASS) {
            esp_err_t err = pi_communicator_send_uid(received_uid);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send UID to RPi: %s", esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG, "Successfully sent UID to RPi.");
            }
            free(received_uid);
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Pet Feeder application starting...");

    rfid_uid_queue = xQueueCreate(RFID_UID_QUEUE_LENGTH, sizeof(RFID_Uid_t*));
    if (rfid_uid_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create RFID UID queue. Aborting.");
        return;
    }

   // if (pi_communicator_init(PI_ENDPOINT_URL) != ESP_OK) {
     //   ESP_LOGE(TAG, "Failed to initialize PI Communicator. Aborting.");
       // return;
//    }

    RFID_Reader_t* rfid_reader_handle = rfid_reader_init(rfid_uid_detected_callback);
    if (rfid_reader_handle == NULL) {
        ESP_LOGE(TAG, "Failed to initialize RFID reader. Aborting.");
        return;
    }

    RFID_Poll_Params_t rfid_poll_params = {
        .rfid_reader = rfid_reader_handle,
        .poll_period_ms = RFID_POLL_PERIOD_MS,
        .task_name = "RFID_Poll_Task"
    };
    xTaskCreate(&rfid_poll_task, rfid_poll_params.task_name, RFID_POLL_TASK_STACK_SIZE, &rfid_poll_params, 5, NULL);

    xTaskCreate(&pi_communicator_task, "PI_Comm_Task", PI_COMM_TASK_STACK_SIZE, NULL, 5, NULL);

    ESP_LOGI(TAG, "Pet Feeder application fully initialized and tasks started.");
}
