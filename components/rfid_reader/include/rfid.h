#ifndef RFID_H
#define RFID_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "pn532.h"

#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

typedef struct RFID_Uid RFID_Uid_t;
typedef struct RFID_Poll_Params RFID_Poll_p;
typedef struct RFID_Reader RFID_Reader_t;
typedef void (*RFID_Uid_Callback_f) (const RFID_Uid_t* uid);

struct RFID_Uid {
    uint8_t uid_len;
    uint8_t uid[];
};

RFID_Reader_t* rfid_reader_init(RFID_Uid_Callback_f uid_cb);

struct RFID_Poll_Params {
    RFID_Reader_t* rfid_reader;
    uint32_t poll_period_ms;
    const char* task_name;
};

void rfid_poll(void* pvParameters);

#endif //RFID_H
