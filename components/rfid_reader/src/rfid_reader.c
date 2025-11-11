#include "rfid_reader.h"
#include "i2c_driver.h"
#include "esp_log.h"
#include <string.h>
#include "driver/gpio.h"

static const char *TAG = "RFID_READER";

#define PN532_RESET_PIN (GPIO_NUM_3)

#define PN532_I2C_ADDRESS               (0x24)
#define PN532_PREAMBLE                  (0x00)
#define PN532_STARTCODE1                (0x00)
#define PN532_STARTCODE2                (0xFF)
#define PN532_POSTAMBLE                 (0x00)

#define PN532_HOSTTOPN532               (0xD4)
#define PN532_PN532TOHOST               (0xD5)

#define PN532_COMMAND_GETFIRMWAREVERSION (0x02)
#define PN532_COMMAND_SAMCONFIGURATION   (0x14)
#define PN532_COMMAND_INLISTPASSIVETARGET (0x4A)

#define PN532_ACK_TIMEOUT_MS            (1000)
#define PN532_COMM_TIMEOUT_MS           (1000)

#define PN532_MIFARE_ISO14443A          (0x00)

static esp_err_t pn532_read_ack(RFID_Reader_t* reader);
static esp_err_t pn532_wait_ready(RFID_Reader_t* reader, uint16_t timeout);
static esp_err_t pn532_write_command(RFID_Reader_t* reader, const uint8_t* cmd, uint8_t cmd_len);
static int pn532_read_response(RFID_Reader_t* reader, uint8_t* buf, uint8_t len, uint16_t timeout);

static esp_err_t pn532_write_ack(RFID_Reader_t* reader) {
    const uint8_t ack[] = {PN532_PREAMBLE, PN532_STARTCODE1, PN532_STARTCODE2, 0x00, 0xFF, PN532_POSTAMBLE};
    return i2c_master_transmit(reader->i2c_dev_handle, ack, sizeof(ack), 100);
}

static esp_err_t pn532_wait_ready(RFID_Reader_t* reader, uint16_t timeout) {
    uint8_t status;
    TickType_t start = xTaskGetTickCount();
    while (xTaskGetTickCount() - start < pdMS_TO_TICKS(timeout)) {
        esp_err_t err = i2c_master_receive(reader->i2c_dev_handle, &status, 1, 100);
        if (err == ESP_OK && (status == 0x01)) {
            return ESP_OK; // Ready bit is set
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t pn532_read_ack(RFID_Reader_t* reader) {
    if (pn532_wait_ready(reader, PN532_ACK_TIMEOUT_MS) != ESP_OK) {
        ESP_LOGE(TAG, "Timed out waiting for ACK/NACK frame");
        return ESP_ERR_TIMEOUT;
    }

    const uint8_t expected_ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
    uint8_t received_ack[sizeof(expected_ack)];

    esp_err_t err = i2c_master_receive(reader->i2c_dev_handle, received_ack, sizeof(received_ack), 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read for ACK failed: %s", esp_err_to_name(err));
        return err;
    }

    if (memcmp(expected_ack, received_ack, sizeof(expected_ack)) != 0) {
        ESP_LOGE(TAG, "Invalid ACK frame received. Expected: 00 00 FF 00 FF 00, Got: %02x %02x %02x %02x %02x %02x",
                 received_ack[0], received_ack[1], received_ack[2],
                 received_ack[3], received_ack[4], received_ack[5]);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t pn532_write_command(RFID_Reader_t* reader, const uint8_t* cmd, uint8_t cmd_len) {
    uint8_t checksum;
    uint8_t frame_len = cmd_len + 1;

    uint8_t buffer[8 + cmd_len];
    buffer[0] = PN532_PREAMBLE;
    buffer[1] = PN532_STARTCODE1;
    buffer[2] = PN532_STARTCODE2;
    buffer[3] = frame_len;
    buffer[4] = ~frame_len + 1;
    buffer[5] = PN532_HOSTTOPN532;

    checksum = PN532_HOSTTOPN532;
    for (uint8_t i = 0; i < cmd_len; i++) {
        buffer[6 + i] = cmd[i];
        checksum += cmd[i];
    }

    buffer[6 + cmd_len] = ~checksum + 1;
    buffer[7 + cmd_len] = PN532_POSTAMBLE;

    esp_err_t err = i2c_master_transmit(reader->i2c_dev_handle, buffer, 8 + cmd_len, PN532_COMM_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C transmit failed: %s", esp_err_to_name(err));
        return err;
    }

    return pn532_read_ack(reader);
}

static int pn532_read_response(RFID_Reader_t* reader, uint8_t* buf, uint8_t len, uint16_t timeout) {
    if (pn532_wait_ready(reader, timeout) != ESP_OK) {
        ESP_LOGD(TAG, "Wait ready timed out");
        return -1;
    }

    uint8_t response_header[6]; // Preamble + Start Code(2) + LEN + LCS + TFI
    esp_err_t err = i2c_master_receive(reader->i2c_dev_handle, response_header, sizeof(response_header), timeout);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read response header: %s", esp_err_to_name(err));
        return -1;
    }

    if (response_header[0] != PN532_PREAMBLE || response_header[1] != PN532_STARTCODE1 || response_header[2] != PN532_STARTCODE2) {
        ESP_LOGE(TAG, "Invalid response frame header");
        return -2;
    }

    uint8_t frame_len = response_header[3];
    if ((uint8_t)(frame_len + response_header[4]) != 0x00) {
        ESP_LOGE(TAG, "Response length checksum failed");
        return -3;
    }

    if (response_header[5] != PN532_PN532TOHOST) {
        ESP_LOGE(TAG, "Invalid response frame identifier");
        return -4;
    }

    int data_len = frame_len - 1;
    if (data_len <= 0) {
        pn532_write_ack(reader);
        return 0;
    }
    if (data_len > len) {
        ESP_LOGE(TAG, "Response buffer too small. Required: %d, Available: %d", data_len, len);
        return -5;
    }

    uint8_t data_buffer[data_len + 2];
    err = i2c_master_receive(reader->i2c_dev_handle, data_buffer, sizeof(data_buffer), timeout);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read response data: %s", esp_err_to_name(err));
        return -6;
    }

    uint8_t checksum = PN532_PN532TOHOST;
    for (int i = 0; i < data_len; i++) {
        buf[i] = data_buffer[i];
        checksum += data_buffer[i];
    }
    checksum += data_buffer[data_len];
    if (checksum != 0x00) {
        ESP_LOGE(TAG, "Response data checksum failed");
        return -7;
    }

    pn532_write_ack(reader);

    return data_len;
}

RFID_Reader_t* rfid_reader_init(RFID_Uid_Callback_f uid_cb) {
    if (uid_cb == NULL) {
        ESP_LOGE(TAG, "Callback function is NULL");
        return NULL;
    }

    RFID_Reader_t* reader = (RFID_Reader_t*) calloc(1, sizeof(RFID_Reader_t));
    if (reader == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for RFID_Reader_t.");
        return NULL;
    }
    reader->uid_callback = uid_cb;
    reader->i2c_dev_handle = NULL;

    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C master initialization failed");
        free(reader);
        return NULL;
    }

    i2c_device_config_t pn532_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PN532_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(g_i2c_bus_handle, &pn532_cfg, &reader->i2c_dev_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add PN532 device to I2C bus");
        i2c_master_deinit();
        free(reader);
        return NULL;
    }

    ESP_LOGI(TAG, "PN532 I2C device added successfully.");

    ESP_LOGI(TAG, "Performing hardware reset on GPIO %d...", PN532_RESET_PIN);
    gpio_config_t reset_gpio_cfg = {
        .pin_bit_mask = (1ULL << PN532_RESET_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&reset_gpio_cfg);

    gpio_set_level(PN532_RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PN532_RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(400));

    uint8_t cmd_fw[] = {PN532_COMMAND_GETFIRMWAREVERSION};
    if (pn532_write_command(reader, cmd_fw, sizeof(cmd_fw)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send GetFirmwareVersion command.");
        goto error_cleanup;
    }

    uint8_t response_buf[12];
    int response_len = pn532_read_response(reader, response_buf, sizeof(response_buf), PN532_COMM_TIMEOUT_MS);
    if (response_len < 5) {
        ESP_LOGE(TAG, "Failed to get firmware version. Response length: %d", response_len);
        goto error_cleanup;
    }

    ESP_LOGI(TAG, "Found PN532 with firmware ver %d.%d", response_buf[2], response_buf[3]);

    uint8_t cmd_sam[] = {PN532_COMMAND_SAMCONFIGURATION, 0x01, 0x14, 0x01};
    if (pn532_write_command(reader, cmd_sam, sizeof(cmd_sam)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send SAMConfiguration command.");
        goto error_cleanup;
    }
    response_len = pn532_read_response(reader, response_buf, sizeof(response_buf), PN532_COMM_TIMEOUT_MS);
    if (response_len < 1 || response_buf[0] != 0x15) {
        ESP_LOGE(TAG, "Failed to configure SAM.");
        goto error_cleanup;
    }

    ESP_LOGI(TAG, "RFID reader initialized successfully.");
    return reader;

error_cleanup:
    if (reader->i2c_dev_handle) i2c_master_bus_rm_device(reader->i2c_dev_handle);
    i2c_master_deinit();
    free(reader);
    return NULL;
}

void rfid_reader_deinit(RFID_Reader_t* reader) {
    if (reader == NULL) {
        return;
    }
    if (reader->i2c_dev_handle != NULL) {
        i2c_master_bus_rm_device(reader->i2c_dev_handle);
    }
    i2c_master_deinit();
    free(reader);
    ESP_LOGI(TAG, "RFID reader de-initialized.");
}

void rfid_poll_task(void* pvParameters) {
    RFID_Poll_Params_t* params = (RFID_Poll_Params_t*)pvParameters;
    RFID_Reader_t* reader = params->rfid_reader;
    uint32_t poll_period_ms = params->poll_period_ms;

    if (reader == NULL || reader->uid_callback == NULL) {
        ESP_LOGE(TAG, "Invalid RFID_Poll_Params_t. Aborting task.");
        vTaskDelete(NULL);
        return;
    }

    RFID_Uid_t* rfid_uid_data = (RFID_Uid_t*)malloc(sizeof(RFID_Uid_t) + 16);
    if (rfid_uid_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for RFID_Uid_t. Aborting task.");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "RFID polling task started. Scanning for cards...");

    while (1) {
        uint8_t cmd_poll[] = {PN532_COMMAND_INLISTPASSIVETARGET, 1, PN532_MIFARE_ISO14443A};
        if (pn532_write_command(reader, cmd_poll, sizeof(cmd_poll)) != ESP_OK) {
             ESP_LOGW(TAG, "Error sending poll command. Retrying...");
             vTaskDelay(pdMS_TO_TICKS(500));
             continue;
        }

        uint8_t response_buf[32];
        int response_len = pn532_read_response(reader, response_buf, sizeof(response_buf), 200);

        if (response_len > 1 && response_buf[0] == (PN532_COMMAND_INLISTPASSIVETARGET + 1) && response_buf[1] == 1) {
            uint8_t uid_len = response_buf[6];
            if (response_len >= 7 + uid_len) {
                rfid_uid_data->uid_len = uid_len;
                memcpy(rfid_uid_data->uid, &response_buf[7], uid_len);

                char uid_str[uid_len * 3 + 1];
                for(int i = 0; i < uid_len; i++) {
                    sprintf(uid_str + i*3, "%02X ", rfid_uid_data->uid[i]);
                }
                ESP_LOGI(TAG, "Card detected! UID: %s", uid_str);

                reader->uid_callback(rfid_uid_data);

                vTaskDelay(pdMS_TO_TICKS(1000));
            } else {
                 ESP_LOGD(TAG, "Card detected but response length is too short. Len: %d, UID Len: %d", response_len, uid_len);
            }
        } else if (response_len < 0) {
            ESP_LOGW(TAG, "Error reading response: %d", response_len);
        }

        vTaskDelay(pdMS_TO_TICKS(poll_period_ms));
    }

    free(rfid_uid_data);
    vTaskDelete(NULL);
}