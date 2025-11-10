#include "rfid_reader.h"
#include "esp_log.h"
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "RFID_READER";

// --- Pin Definitions (CORRECTED to match available header pins) ---
#define PN532_HOST      SPI2_HOST
#define PIN_NUM_MISO    GPIO_NUM_0
#define PIN_NUM_MOSI    GPIO_NUM_1
#define PIN_NUM_CLK     GPIO_NUM_3
#define PIN_NUM_CS      GPIO_NUM_9
#define PIN_NUM_RST     GPIO_NUM_10
// --- PN532 Constants ---
#define PN532_PREAMBLE                  (0x00)
#define PN532_STARTCODE1                (0x00)
#define PN532_STARTCODE2                (0xFF)
#define PN532_POSTAMBLE                 (0x00)
#define PN532_HOSTTOPN532               (0xD4)
#define PN532_PN532TOHOST               (0xD5)

#define PN532_COMMAND_GETFIRMWAREVERSION (0x02)
#define PN532_COMMAND_SAMCONFIGURATION   (0x14)
#define PN532_COMMAND_INLISTPASSIVETARGET (0x4A)

#define PN532_ACK_FRAME                 (0x01)
#define PN532_FRAME_START_SIZE          (7)
#define PN532_ACK_TIMEOUT_MS            (1000)
#define PN532_COMM_TIMEOUT_MS           (1000)

// --- Local Function Prototypes ---
static esp_err_t pn532_write_command(RFID_Reader_t* reader, const uint8_t* cmd, uint8_t cmd_len);
static int pn532_read_response(RFID_Reader_t* reader, uint8_t* buf, uint8_t len, uint16_t timeout);


static void pn532_wakeup(void) {
    gpio_set_level(PIN_NUM_CS, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_NUM_CS, 1);
}

static esp_err_t pn532_write_command(RFID_Reader_t* reader, const uint8_t* cmd, uint8_t cmd_len) {
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
    frame[7 + cmd_len] = PN532_POSTAMBLE;

    spi_transaction_t t = {
        .length = sizeof(frame) * 8, // length in bits
        .tx_buffer = frame,
    };

    return spi_device_polling_transmit(reader->spi_handle, &t);
}

static int pn532_read_response(RFID_Reader_t* reader, uint8_t* buf, uint8_t len, uint16_t timeout) {
    uint8_t frame[len + 2]; // Room for status and the data
    TickType_t start = xTaskGetTickCount();

    // Poll for the status byte to be ready
    while (xTaskGetTickCount() - start < pdMS_TO_TICKS(timeout)) {
        spi_transaction_t t_status = {
            .length = 8,
            .flags = SPI_TRANS_USE_RXDATA,
        };
        spi_device_polling_transmit(reader->spi_handle, &t_status);
        if (t_status.rx_data[0] == 0x01) { // 0x01 is the READY status for SPI
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (xTaskGetTickCount() - start >= pdMS_TO_TICKS(timeout)) {
        return -1; // Timeout
    }

    // Read the full response
    spi_transaction_t t = {
        .length = len * 8,
        .rx_buffer = frame,
    };
    spi_device_polling_transmit(reader->spi_handle, &t);

    if (frame[0] != PN532_PREAMBLE || frame[1] != PN532_STARTCODE1 || frame[2] != PN532_STARTCODE2) {
        ESP_LOGE(TAG, "Invalid response frame header");
        return -2;
    }

    uint8_t frame_len = frame[3];
    if ((uint8_t)(frame_len + frame[4]) != 0) {
        ESP_LOGE(TAG, "Response length checksum failed");
        return -3;
    }

    if (frame[5] != PN532_PN532TOHOST) {
        ESP_LOGE(TAG, "Invalid response frame identifier");
        return -4;
    }

    int data_len = frame_len - 1;
    memcpy(buf, &frame[6], data_len);

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
    /*
    // --- Hardware Reset ---
    ESP_LOGI(TAG, "Performing hardware reset on GPIO %d...", PIN_NUM_RST);
    gpio_config_t reset_gpio_cfg = { .pin_bit_mask = (1ULL << PIN_NUM_RST), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&reset_gpio_cfg);
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(400));
    */
    // --- Initialize SPI Bus ---
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    esp_err_t ret = spi_bus_initialize(PN532_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        free(reader);
        return NULL;
    }

    // --- Add PN532 as a device on the bus ---
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };
    ret = spi_bus_add_device(PN532_HOST, &devcfg, &reader->spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        spi_bus_free(PN532_HOST);
        free(reader);
        return NULL;
    }

    pn532_wakeup();

    // --- Get Firmware Version ---
    uint8_t cmd_fw[] = {PN532_COMMAND_GETFIRMWAREVERSION};
    if (pn532_write_command(reader, cmd_fw, sizeof(cmd_fw)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send GetFirmwareVersion command.");
        goto error_cleanup;
    }

    uint8_t response_buf[12];
    int response_len = pn532_read_response(reader, response_buf, sizeof(response_buf), PN532_ACK_TIMEOUT_MS);
    if (response_len < 4) {
        ESP_LOGE(TAG, "Failed to get firmware version. Response length: %d", response_len);
        goto error_cleanup;
    }
    ESP_LOGI(TAG, "Found PN532 with firmware ver %d.%d", response_buf[1], response_buf[2]);

    // --- Configure SAM ---
    uint8_t cmd_sam[] = {PN532_COMMAND_SAMCONFIGURATION, 0x01, 0x14, 0x01};
    if (pn532_write_command(reader, cmd_sam, sizeof(cmd_sam)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send SAMConfiguration command.");
        goto error_cleanup;
    }
    response_len = pn532_read_response(reader, response_buf, sizeof(response_buf), PN532_ACK_TIMEOUT_MS);
    if (response_len < 0 || response_buf[0] != 0x15) {
        ESP_LOGE(TAG, "Failed to configure SAM.");
        goto error_cleanup;
    }

    ESP_LOGI(TAG, "RFID reader initialized successfully.");
    return reader;

error_cleanup:
    spi_bus_remove_device(reader->spi_handle);
    spi_bus_free(PN532_HOST);
    free(reader);
    return NULL;
}


void rfid_reader_deinit(RFID_Reader_t* reader) {
    if (reader == NULL) {
        return;
    }
    spi_bus_remove_device(reader->spi_handle);
    spi_bus_free(PN532_HOST);
    free(reader);
    ESP_LOGI(TAG, "RFID reader de-initialized.");
}


void rfid_poll_task(void* pvParameters) {
    RFID_Poll_Params_t* params = (RFID_Poll_Params_t*)pvParameters;
    RFID_Reader_t* reader = params->rfid_reader;
    uint32_t poll_period_ms = params->poll_period_ms;

    RFID_Uid_t* rfid_uid_data = (RFID_Uid_t*)malloc(sizeof(RFID_Uid_t) + 16);
    if (rfid_uid_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for RFID_Uid_t. Aborting task.");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "RFID polling task started. Scanning for cards...");

    while (1) {
        uint8_t cmd_poll[] = {PN532_COMMAND_INLISTPASSIVETARGET, 1, 0x00};
        if (pn532_write_command(reader, cmd_poll, sizeof(cmd_poll)) != ESP_OK) {
             ESP_LOGW(TAG, "Error sending poll command.");
             vTaskDelay(pdMS_TO_TICKS(500));
             continue;
        }

        uint8_t response_buf[32];
        int response_len = pn532_read_response(reader, response_buf, sizeof(response_buf), 1000);

        if (response_len > 0 && response_buf[0] == 0x01) { // NbTg = 1
            uint8_t uid_len = response_buf[5];
            rfid_uid_data->uid_len = uid_len;
            memcpy(rfid_uid_data->uid, &response_buf[6], uid_len);

            char uid_str[uid_len * 3 + 1];
            for(int i = 0; i < uid_len; i++) {
                sprintf(uid_str + i*3, "%02X ", rfid_uid_data->uid[i]);
            }
            ESP_LOGI(TAG, "Card detected! UID: %s", uid_str);

            reader->uid_callback(rfid_uid_data);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        vTaskDelay(pdMS_TO_TICKS(poll_period_ms));
    }
    free(rfid_uid_data);
    vTaskDelete(NULL);
}