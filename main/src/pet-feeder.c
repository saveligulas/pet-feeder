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

// We don't use these in the init test, but they are needed for compilation
#define RFID_POLL_PERIOD_MS    500
#define PI_COMM_TASK_STACK_SIZE 4096
#define RFID_POLL_TASK_STACK_SIZE 3072
#define RFID_UID_QUEUE_LENGTH  10

// --- Debugging Constants for the Test ---
#define RFID_INIT_RETRY_COUNT  5
#define RFID_INIT_RETRY_DELAY_MS 2000 // 2 second delay between retries

// These are not used in the init test, but the functions must exist for the code to compile
static QueueHandle_t rfid_uid_queue;
static void rfid_uid_detected_callback(const RFID_Uid_t* uid) { /* Dummy for now */ }
static void pi_communicator_task(void* pvParameters) { while(1) vTaskDelay(10000); }


void app_main(void) {
    ESP_LOGI(TAG, "Pet Feeder application starting...");
    ESP_LOGI(TAG, "--- FINAL TEST: Using physically verified GPIOs for UART ---");


    // The queue isn't used in the init test, but we create it so the code is complete
    rfid_uid_queue = xQueueCreate(RFID_UID_QUEUE_LENGTH, sizeof(RFID_Uid_t*));
    if (rfid_uid_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create RFID UID queue. Aborting.");
        return;
    }

    // --- Initialization with Retry Logic ---
    RFID_Reader_t* rfid_reader_handle = NULL;

    for (int i = 0; i < RFID_INIT_RETRY_COUNT; i++) {
        ESP_LOGI(TAG, "Attempting to initialize RFID reader (Attempt %d/%d)...", i + 1, RFID_INIT_RETRY_COUNT);

        // This is the function we are testing
        rfid_reader_handle = rfid_reader_init(rfid_uid_detected_callback);

        if (rfid_reader_handle != NULL) {
            // If rfid_reader_init() returns a valid handle, it means success!
            ESP_LOGI(TAG, "Initialization attempt SUCCEEDED!");
            break; // Exit the loop
        }

        // If it returned NULL, it failed.
        ESP_LOGE(TAG, "Initialization attempt failed. Retrying in %dms...", RFID_INIT_RETRY_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(RFID_INIT_RETRY_DELAY_MS));
    }

    // --- Final Check ---
    if (rfid_reader_handle == NULL) {
        ESP_LOGE(TAG, "Failed to initialize RFID reader after %d attempts. Halting.", RFID_INIT_RETRY_COUNT);
        // In a real product, you might want to blink an error LED here.
        return; // Abort the application
    }

    // If we get here, the RFID reader is initialized and ready.
    ESP_LOGI(TAG, "Creating tasks...");

    RFID_Poll_Params_t rfid_poll_params = {
        .rfid_reader = rfid_reader_handle,
        .poll_period_ms = RFID_POLL_PERIOD_MS,
        .task_name = "RFID_Poll_Task"
    };
    // The poll task in our test code will just print a message and halt.
    xTaskCreate(rfid_poll_task, rfid_poll_params.task_name, RFID_POLL_TASK_STACK_SIZE, &rfid_poll_params, 5, NULL);

    // This task will just block, which is fine for our test.
    xTaskCreate(pi_communicator_task, "PI_Comm_Task", PI_COMM_TASK_STACK_SIZE, NULL, 5, NULL);

    ESP_LOGI(TAG, "Application fully initialized. Test successful.");
}