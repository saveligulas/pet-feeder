#ifndef RFID_H
#define RFID_H

#define SDA_PIN 6
#define SCL_PIN 7
#include <stdint.h>

typedef struct RFID_Uid RFID_Uid_t;
typedef struct RFID_Reader RFID_Reader_t;
typedef void (*RFID_Uid_Callback_f) (const RFID_Uid_t* uid);

struct RFID_Uid {
    uint8_t uid_len;
    uint8_t uid[];
};

RFID_Reader_t* rfid_reader_init(RFID_Uid_Callback_f uid_cb);


#endif //RFID_H
