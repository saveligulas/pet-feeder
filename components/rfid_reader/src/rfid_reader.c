#include "rfid.h"
#include "rfid_reader.h"
#include "i2c_driver.h"
#include "esp_log.h"
#include <string.h>

#include "pn532_driver_i2c.h"

static const char *TAG = "RFID_READER";

RFID_Reader_t* rfid_reader_init(RFID_Uid_Callback_f uid_cb) {
    if (uid_cb == NULL) return NULL;

    RFID_Reader_t* reader = (RFID_Reader_t*) calloc(1, sizeof(RFID_Reader_t));
    if (reader == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for RFID_Reader_t.");
        return NULL;
    }

    reader->uid_callback = uid_cb;

    if (i2c_master_init() != ESP_OK) {
        free(reader);
        return NULL;
    }

    ESP_ERROR_CHECK(pn532_new_driver_i2c(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, -1, -1, I2C_MASTER_NUM, &reader->pn532_io));

    esp_err_t err = pn532_init(&reader->pn532_io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PN532.");
        pn532_release(&reader->pn532_io);
        i2c_master_deinit();
        free(reader);
        return NULL;
    }

    uint32_t versiondata;
    if (pn532_get_firmware_version(&reader->pn532_io, &versiondata) != ESP_OK) {
        ESP_LOGE(TAG, "PN532 not found. Check connections.");
        pn532_release(&reader->pn532_io);
        i2c_master_deinit();
        free(reader);
        return NULL;
    }

    ESP_LOGI(TAG, "Found PN532, Firmware ver. %d.%d", (int)(versiondata >> 16) & 0xFF, (int)(versiondata >> 8) & 0xFF);
    pn532_set_passive_activation_retries(&reader->pn532_io, 0xFF); // Try forever

    ESP_LOGI(TAG, "RFID reader initialized.");
    return reader;
}

void rfid_reader_deinit(RFID_Reader_t* reader) {
    if (reader == NULL) return;
    pn532_release(&reader->pn532_io);
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
    }

    uint8_t uid[16];
    uint8_t uid_len;
    esp_err_t err;

    RFID_Uid_t* rfid_uid_data = (RFID_Uid_t*)malloc(sizeof(RFID_Uid_t) + 16);
    if (rfid_uid_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for RFID_Uid_t. Aborting task.");
        vTaskDelete(NULL);
    }

    while (1) {
        err = pn532_read_passive_target_id(&reader->pn532_io,
                                           PN532_BRTY_ISO14443A_106KBPS,
                                           uid,
                                           &uid_len,
                                           0);

        if (err == ESP_OK && uid_len > 0) {
            rfid_uid_data->uid_len = uid_len;
            memcpy(rfid_uid_data->uid, uid, uid_len);
            reader->uid_callback(rfid_uid_data);
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGD(TAG, "Error reading passive target: %s", esp_err_to_name(err));
            pn532_init(&reader->pn532_io);
        }

        vTaskDelay(pdMS_TO_TICKS(poll_period_ms));
    }

    free(rfid_uid_data);
    vTaskDelete(NULL);
}
