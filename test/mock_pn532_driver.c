#include "pn532.h"
#include "pn532_driver.h"
#include "pn532_driver_i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "MOCK_PN532";

static uint8_t mock_uid_1[] = { 0xDE, 0xAD, 0xBE, 0xEF };
static bool tag_present_toggle = false;

esp_err_t pn532_new_driver_i2c(gpio_num_t sda,
                               gpio_num_t scl,
                               gpio_num_t reset,
                               gpio_num_t irq,
                               i2c_port_num_t i2c_port_number,
                               pn532_io_handle_t io_handle)
{
    ESP_LOGI(TAG, "MOCK: pn532_new_driver_i2c called. Creating dummy handle.");
    return ESP_OK;
}

esp_err_t pn532_init(pn532_io_handle_t io_handle) {
    ESP_LOGI(TAG, "MOCK: pn532_init called.");
    return ESP_OK;
}

void pn532_release(pn532_io_handle_t io_handle) {
    ESP_LOGI(TAG, "MOCK: pn532_release called.");
}

esp_err_t pn532_get_firmware_version(pn532_io_handle_t io_handle, uint32_t *fw_version) {
    ESP_LOGI(TAG, "MOCK: pn532_get_firmware_version called.");
    *fw_version = 0x01020304;
    return ESP_OK;
}

esp_err_t pn532_set_passive_activation_retries(pn532_io_handle_t io_handle, uint8_t maxRetries) {
    ESP_LOGI(TAG, "MOCK: pn532_set_passive_activation_retries called with %d.", maxRetries);
    return ESP_OK;
}

esp_err_t pn532_read_passive_target_id(pn532_io_handle_t io_handle,
                                       uint8_t baud_rate_and_card_type,
                                       uint8_t *uid,
                                       uint8_t *uid_length,
                                       int32_t timeout) {
    vTaskDelay(pdMS_TO_TICKS(500));

    tag_present_toggle = !tag_present_toggle;
    if (tag_present_toggle) {
        *uid_length = sizeof(mock_uid_1);
        memcpy(uid, mock_uid_1, *uid_length);
        ESP_LOGI(TAG, "MOCK: >>> Tag Detected! UID: DEADBEEF <<<");
        return ESP_OK;
    } else {
        ESP_LOGD(TAG, "MOCK: No tag detected.");
        *uid_length = 0;
        return ESP_ERR_TIMEOUT;
    }
}
