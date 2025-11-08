#include "rfid_reader.h"
#include "esp_log.h"
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "RFID_READER";

// --- Pin Definitions Based on YOUR Physical "Pin Finder" Test ---
#define PN532_UART_PORT     UART_NUM_1
#define PIN_NUM_TX          GPIO_NUM_0   // Your proven GPIO 0 pin (ESP TX)
#define PIN_NUM_RX          GPIO_NUM_1   // Your proven GPIO 1 pin (ESP RX)
#define PIN_NUM_RST         GPIO_NUM_9   // Your proven GPIO 3 pin

#define UART_BAUD_RATE      115200
#define UART_BUF_SIZE       (256)

// --- PN532 Constants ---
#define PN532_PREAMBLE                  (0x00)
#define PN532_STARTCODE1                (0x00)
#define PN532_STARTCODE2                (0xFF)
#define PN532_HOSTTOPN532               (0xD4)
#define PN532_PN532TOHOST               (0xD5)
#define PN532_COMMAND_SAMCONFIGURATION   (0x14)
#define PN532_COMMAND_INLISTPASSIVETARGET (0x4A)
#define PN532_ACK_TIMEOUT_MS            (1000)


static esp_err_t pn532_write_command(const uint8_t* cmd, uint8_t cmd_len) {
    uint8_t frame[cmd_len + 8];
    frame[0] = PN532_PREAMBLE;
    frame[1] = PN532_STARTCODE1;
    frame[2] = PN532_STARTCODE2;
    uint8_t len = cmd_len + 1;
    frame[3] = len;
    frame[4] = ~len + 1;
    frame[5] = PN532_HOSTTOPN532;

    uint8_t sum = PN532_HOSTTOPN532;
    for (uint8_t i = 0; i < cmd_len; i++) {
        frame[6 + i] = cmd[i];
        sum += cmd[i];
    }
    frame[6 + cmd_len] = ~sum + 1;
    frame[7] = 0x00; // Postamble

    uart_write_bytes(PN532_UART_PORT, frame, sizeof(frame));

    uint8_t ack_buf[6];
    int read_len = uart_read_bytes(PN532_UART_PORT, ack_buf, sizeof(ack_buf), pdMS_TO_TICKS(PN532_ACK_TIMEOUT_MS));
    if (read_len != 6 || ack_buf[0] != 0x00 || ack_buf[1] != 0x00 || ack_buf[2] != 0xFF || ack_buf[3] != 0x00 || ack_buf[4] != 0xFF || ack_buf[5] != 0x00) {
        ESP_LOGE(TAG, "Invalid ACK received. Read %d bytes.", read_len);
        if(read_len > 0) ESP_LOG_BUFFER_HEX(TAG, ack_buf, read_len);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static int pn532_read_response(uint8_t* buf, uint8_t len) {
    uint8_t header[6];
    int read_len = uart_read_bytes(PN532_UART_PORT, header, sizeof(header), pdMS_TO_TICKS(PN532_ACK_TIMEOUT_MS));
    if (read_len < 6) {
        ESP_LOGE(TAG, "Read response header failed, read %d bytes", read_len);
        return -1;
    }

    uint8_t frame_len = header[3];
    if ((uint8_t)(frame_len + header[4]) != 0) {
        ESP_LOGE(TAG, "Read response length checksum failed");
        return -2;
    }

    int data_len = frame_len;
    if (data_len > len) {
        ESP_LOGE(TAG, "Read response buffer too small");
        return -3;
    }

    read_len = uart_read_bytes(PN532_UART_PORT, buf, data_len + 1, pdMS_TO_TICKS(PN532_ACK_TIMEOUT_MS));
    if (read_len < data_len + 1) {
        ESP_LOGE(TAG, "Read response data failed");
        return -4;
    }

    // We are not validating the data checksum here for simplicity, but it's good enough for this test
    return data_len;
}

RFID_Reader_t* rfid_reader_init(RFID_Uid_Callback_f uid_cb) {
    RFID_Reader_t* reader = (RFID_Reader_t*) calloc(1, sizeof(RFID_Reader_t));
    if (!reader) { ESP_LOGE(TAG, "calloc failed"); return NULL; }
    reader->uid_callback = uid_cb;

    ESP_LOGI(TAG, "Performing hardware reset on GPIO %d...", PIN_NUM_RST);
    gpio_config_t reset_gpio_cfg = { .pin_bit_mask = (1ULL << PIN_NUM_RST), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&reset_gpio_cfg);
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(500));

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_delete(PN532_UART_PORT);
    ESP_ERROR_CHECK(uart_driver_install(PN532_UART_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PN532_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(PN532_UART_PORT, PIN_NUM_TX, PIN_NUM_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    vTaskDelay(pdMS_TO_TICKS(100));
    uart_flush_input(PN532_UART_PORT);

    ESP_LOGI(TAG, "Sending SAMConfiguration command...");
    uint8_t cmd_sam[] = {PN532_COMMAND_SAMCONFIGURATION, 0x01, 0x14, 0x01};
    // CORRECTED FUNCTION CALL
    if (pn532_write_command(cmd_sam, sizeof(cmd_sam)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure SAM.");
        uart_driver_delete(PN532_UART_PORT);
        free(reader);
        return NULL;
    }

    ESP_LOGI(TAG, ">>> SUCCESS! Received ACK from PN532. Communication is working. <<<");
    return reader;
}

void rfid_reader_deinit(RFID_Reader_t* reader) {
    if (!reader) return;
    uart_driver_delete(PN532_UART_PORT);
    free(reader);
}

void rfid_poll_task(void* pvParameters) {
    RFID_Poll_Params_t* params = (RFID_Poll_Params_t*)pvParameters;
    RFID_Uid_t* rfid_uid_data = (RFID_Uid_t*)malloc(sizeof(RFID_Uid_t) + 16);
    ESP_LOGI(TAG, "RFID polling task started.");

    while (1) {
        uint8_t cmd_poll[] = {PN532_COMMAND_INLISTPASSIVETARGET, 1, 0x00};
        // CORRECTED FUNCTION CALL
        if (pn532_write_command(cmd_poll, sizeof(cmd_poll)) == ESP_OK) {
            uint8_t response_buf[32];
            int response_len = pn532_read_response(response_buf, sizeof(response_buf));

            if (response_len > 8 && response_buf[1] == 0x4B && response_buf[2] == 1) { // Check for CMD+1 and NbTg=1
                uint8_t uid_len = response_buf[7];
                if (response_len >= 8 + uid_len) {
                    rfid_uid_data->uid_len = uid_len;
                    memcpy(rfid_uid_data->uid, &response_buf[8], uid_len);

                    char uid_str[uid_len * 3 + 1];
                    for(int i = 0; i < uid_len; i++) {
                        sprintf(uid_str + i*3, "%02X ", rfid_uid_data->uid[i]);
                    }
                    ESP_LOGI(TAG, "Card detected! UID: %s", uid_str);
                    params->rfid_reader->uid_callback(rfid_uid_data);
                    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(params->poll_period_ms));
    }
}