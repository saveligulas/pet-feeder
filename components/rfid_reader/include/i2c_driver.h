#ifndef I2C_H
#define I2C_H

#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#define I2C_MASTER_SCL_IO           6
#define I2C_MASTER_SDA_IO           5
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TIMEOUT_MS       1000

// Global I2C bus handle - will be used by RFID reader
extern i2c_master_bus_handle_t g_i2c_bus_handle;

esp_err_t i2c_master_init(void);
esp_err_t i2c_master_deinit(void);

#endif //I2C_H