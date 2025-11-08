#ifndef RFID_READER_H
#define RFID_READER_H

#include "rfid.h"
#include "pn532_driver.h"
#include "driver/i2c_master.h"

#include <stdint.h>

struct RFID_Reader {
    pn532_io_t pn532_io;
    RFID_Uid_Callback_f uid_callback;
    i2c_master_dev_handle_t i2c_dev_handle;  // Add device handle for new I2C API
};

#endif //RFID_READER_H