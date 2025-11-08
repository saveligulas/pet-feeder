#ifndef RFID_READER_H
#define RFID_READER_H

#include "rfid.h"
#include "driver/i2c_master.h"

#include <stdint.h>

struct RFID_Reader {
    RFID_Uid_Callback_f uid_callback;
    i2c_master_dev_handle_t i2c_dev_handle;
};

#endif //RFID_READER_H