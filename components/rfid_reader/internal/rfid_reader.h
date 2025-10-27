#ifndef RFID_READER_H
#define RFID_READER_H

#include <rfid.h>
#include "hal/i2c_types.h"
#include "soc/gpio_num.h"


#include <stdint.h>

typedef struct {
    pn532_t pn532;           // the PN532 driver handle
    i2c_port_t i2c_port;     // which I2C controller (e.g., I2C_NUM_0)
    gpio_num_t sda_pin;      // SDA GPIO
    gpio_num_t scl_pin;      // SCL GPIO
    uint32_t clk_speed;      // I2C frequency
} RFID_Reader;

#endif //RFID_READER_H
