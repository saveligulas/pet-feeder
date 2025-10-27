#include "i2c_driver.h" // CORRECTED PATH
#include "esp_log.h"

static const char *TAG = "I2C_DRIVER";

esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "I2C master initialized.");
    return ESP_OK;
}

esp_err_t i2c_master_deinit(void) {
    esp_err_t err = i2c_driver_delete(I2C_MASTER_NUM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver de-installation failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "I2C master de-initialized.");
    return ESP_OK;
}
