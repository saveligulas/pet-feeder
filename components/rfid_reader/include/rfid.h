#ifndef RFID_H
#define RFID_H

#include "pn532.h"
#include "pn532_i2c.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

typedef struct RFID_Uid RFID_Uid_t;
typedef struct RFID_Reader RFID_Reader_t;
typedef void (*RFID_Uid_Callback_f) (const RFID_Uid_t* uid);

struct RFID_Uid {
    uint8_t uid_len;
    uint8_t uid[];
};

RFID_Reader_t* rfid_reader_init(RFID_Uid_Callback_f uid_cb);
void rfid_poll(RFID_Reader_t);

#endif //RFID_H
