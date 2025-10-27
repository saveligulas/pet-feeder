#ifndef PI_COMMUNICATOR_H
#define PI_COMMUNICATOR_H

#include "esp_err.h"
#include <stdint.h>
#include "rfid.h"

esp_err_t pi_communicator_init(const char* pi_endpoint_url);

esp_err_t pi_communicator_send_uid(const RFID_Uid_t* uid);

#endif // PI_COMMUNICATOR_H
