#ifndef RFID_READER_H
#define RFID_READER_H

#include "rfid.h"
#include "pn532_driver.h"

#include <stdint.h>

struct RFID_Reader {
    pn532_io_t pn532_io;
    RFID_Uid_Callback_f uid_callback;
};

#endif //RFID_READER_H
