#ifndef RFID_READER_H
#define RFID_READER_H

#include "rfid.h"
#include "driver/spi_master.h" // Use SPI header
#include <stdint.h>

struct RFID_Reader {
    RFID_Uid_Callback_f uid_callback;
    spi_device_handle_t spi_handle; // Store the SPI device handle
};

#endif //RFID_READER_H