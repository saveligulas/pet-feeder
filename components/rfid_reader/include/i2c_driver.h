#ifndef I2C_H
#define I2C_H

#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

esp_err_t i2c_master_init(void);
esp_err_t i2c_master_deinit(void);

#endif //I2C_H
