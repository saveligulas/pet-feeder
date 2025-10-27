#ifndef RFID_H
#define RFID_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "pn532.h"

typedef struct {
    uint8_t uid_len;
    uint8_t uid[];
} RFID_Uid_t;

typedef struct RFID_Reader RFID_Reader_t;

typedef void (*RFID_Uid_Callback_f)(const RFID_Uid_t* uid);

RFID_Reader_t* rfid_reader_init(RFID_Uid_Callback_f uid_cb);
void rfid_reader_deinit(RFID_Reader_t* reader);

typedef struct {
    RFID_Reader_t* rfid_reader;
    uint32_t poll_period_ms;
    const char* task_name;
} RFID_Poll_Params_t;

void rfid_poll_task(void* pvParameters);

#endif //RFID_H
