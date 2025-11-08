#include "rfid.h"
#include "rfid_reader.h"
#include "i2c_driver.h"
#include "esp_log.h"
#include <string.h>

#include "pn532_driver_i2c.h"

static const char *TAG = "RFID_READER";

// PN532 I2C address (typically 0x24)
#define PN532_I2C_ADDRESS 0x24

RFID_Reader_t* rfid_reader_init(RFID_Uid_Callback_f uid_cb) {
    if (uid_cb == NULL) {
        ESP_LOGE(TAG, "Callback function is NULL");
        return NULL;
    }

    RFID_Reader_t* reader = (RFID_Reader_t*) calloc(1, sizeof(RFID_Reader_t));
    if (reader == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for RFID_Reader_t.");
        return NULL;
    }

    reader->uid_callback = uid_cb;

    // Initialize I2C master bus
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C master initialization failed");
        free(reader);
        return NULL;
    }

    // Check if I2C bus was created successfully
    if (g_i2c_bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus handle is NULL after initialization");
        free(reader);
        return NULL;
    }

    // Configure I2C device for PN532
    i2c_device_config_t pn532_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PN532_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    // Add PN532 device to the I2C bus
    err = i2c_master_bus_add_device(g_i2c_bus_handle, &pn532_cfg, &reader->i2c_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add PN532 device to I2C bus: %s", esp_err_to_name(err));
        i2c_master_deinit();
        free(reader);
        return NULL;
    }

    ESP_LOGI(TAG, "PN532 I2C device added successfully at address 0x%02X", PN532_I2C_ADDRESS);

    // Initialize PN532 driver with I2C
    // Note: The pn532_new_driver_i2c function might need to be updated to work with new I2C API
    // If garag__esp-idf-pn532 doesn't support the new I2C API, you may need to use a different approach
    err = pn532_new_driver_i2c(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, -1, -1, I2C_NUM_0, &reader->pn532_io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PN532 I2C driver: %s", esp_err_to_name(err));
        i2c_master_bus_remove_device(reader->i2c_dev_handle);
        i2c_master_deinit();
        free(reader);
        return NULL;
    }

    // Initialize PN532
    err = pn532_init(&reader->pn532_io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PN532: %s", esp_err_to_name(err));
        pn532_release(&reader->pn532_io);
        i2c_master_bus_remove_device(reader->i2c_dev_handle);
        i2c_master_deinit();
        free(reader);
        return NULL;
    }

    // Get firmware version to verify communication
    uint32_t versiondata;
    err = pn532_get_firmware_version(&reader->pn532_io, &versiondata);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PN532 not found. Check connections and I2C address. Error: %s", esp_err_to_name(err));
        pn532_release(&reader->pn532_io);
        i2c_master_bus_remove_device(reader->i2c_dev_handle);
        i2c_master_deinit();
        free(reader);
        return NULL;
    }

    ESP_LOGI(TAG, "Found PN532, Firmware ver. %d.%d", (int)(versiondata >> 16) & 0xFF, (int)(versiondata >> 8) & 0xFF);

    // Set passive activation retries
    pn532_set_passive_activation_retries(&reader->pn532_io, 0xFF); // Try forever

    ESP_LOGI(TAG, "RFID reader initialized successfully.");
    return reader;
}

void rfid_reader_deinit(RFID_Reader_t* reader) {
    if (reader == NULL) {
        ESP_LOGW(TAG, "Reader is NULL, nothing to deinitialize");
        return;
    }

    // Release PN532 driver
    pn532_release(&reader->pn532_io);

    // Remove I2C device
    if (reader->i2c_dev_handle != NULL) {
        i2c_master_bus_remove_device(reader->i2c_dev_handle);
    }

    // Deinitialize I2C master
    i2c_master_deinit();

    free(reader);
    ESP_LOGI(TAG, "RFID reader de-initialized.");
}

void rfid_poll_task(void* pvParameters) {
    RFID_Poll_Params_t* params = (RFID_Poll_Params_t*)pvParameters;
    RFID_Reader_t* reader = params->rfid_reader;
    uint32_t poll_period_ms = params->poll_period_ms;

    if (reader == NULL || reader->uid_callback == NULL) {
        ESP_LOGE(TAG, "Invalid RFID_Poll_Params_t. Aborting task.");
        vTaskDelete(NULL);
        return;
    }

    uint8_t uid[16];
    uint8_t uid_len;
    esp_err_t err;

    RFID_Uid_t* rfid_uid_data = (RFID_Uid_t*)malloc(sizeof(RFID_Uid_t) + 16);
    if (rfid_uid_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for RFID_Uid_t. Aborting task.");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "RFID polling task started. Scanning for cards...");

    while (1) {
        err = pn532_read_passive_target_id(&reader->pn532_io,
                                           PN532_BRTY_ISO14443A_106KBPS,
                                           uid,
                                           &uid_len,
                                           0);

        if (err == ESP_OK && uid_len > 0) {
            rfid_uid_data->uid_len = uid_len;
            memcpy(rfid_uid_data->uid, uid, uid_len);

            ESP_LOGI(TAG, "Card detected! UID length: %d bytes", uid_len);

            reader->uid_callback(rfid_uid_data);

            // Wait 1 second after successful read to avoid repeated reads of the same card
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGD(TAG, "Error reading passive target: %s", esp_err_to_name(err));

            // Try to reinitialize PN532 if there's a communication error
            if (err == ESP_ERR_INVALID_RESPONSE || err == ESP_FAIL) {
                ESP_LOGW(TAG, "Communication error, attempting to reinitialize PN532...");
                pn532_init(&reader->pn532_io);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(poll_period_ms));
    }

    free(rfid_uid_data);
    vTaskDelete(NULL);
}