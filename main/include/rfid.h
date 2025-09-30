#ifndef RFID_H
#define RFID_H

#define SDA_PIN 6
#define SCL_PIN 7
#include <stdint.h>

typedef struct RFID_Uid RFID_Uid_t;
typedef struct RFID_Reader RFID_Reader_t;

struct RFID_Uid {
    uint8_t uid[];
    uint8_t uid_len;
};

RFID_Reader_t* rfid_reader_init();


#endif //RFID_H
