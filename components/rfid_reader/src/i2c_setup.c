#include "i2c_driver.h"
#include "esp_log.h"

static const char *TAG = "I2C_DRIVER";

i2c_master_bus_handle_t g_i2c_bus_handle = NULL;

esp_err_t i2c_master_init(void) {
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &g_i2c_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus creation failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C master initialized on port %d (SCL: GPIO%d, SDA: GPIO%d)",
             I2C_NUM_0, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    return ESP_OK;
}

esp_err_t i2c_master_deinit(void) {
    if (g_i2c_bus_handle == NULL) {
        ESP_LOGW(TAG, "I2C bus handle is NULL, nothing to deinitialize");
        return ESP_OK;
    }

    esp_err_t err = i2c_del_master_bus(g_i2c_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus deletion failed: %s", esp_err_to_name(err));
        return err;
    }

    g_i2c_bus_handle = NULL;
    ESP_LOGI(TAG, "I2C master de-initialized.");
    return ESP_OK;
}