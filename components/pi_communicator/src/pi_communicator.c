#include "pi_communicator.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "cJSON.h"

static const char *TAG = "PI_COMMUNICATOR";

#define WIFI_SSID           "YOUR_WIFI_SSID" // CRITICAL: Replace with your Wi-Fi SSID
#define WIFI_PASSWORD       "YOUR_WIFI_PASSWORD" // CRITICAL: Replace with your Wi-Fi Password
#define MAX_WIFI_RETRIES    10
#define MAX_HTTP_RECV_BUFFER 512

static char *s_pi_endpoint_url = NULL;
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_WIFI_RETRIES) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retrying Wi-Fi connection...");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Failed to connect to Wi-Fi after max retries.");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    static char *output_buffer;
    static int output_len;
    switch(evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (!esp_http_client_is_chunked_response(evt->client)) {
                if (evt->user_data) {
                    memcpy(evt->user_data + output_len, evt->data, evt->data_len);
                } else {
                    if (output_buffer == NULL) {
                        output_buffer = (char *) malloc(MAX_HTTP_RECV_BUFFER);
                        output_len = 0;
                        if (output_buffer == NULL) {
                            ESP_LOGE(TAG, "Failed to allocate output buffer.");
                            return ESP_FAIL;
                        }
                    }
                    memcpy(output_buffer + output_len, evt->data, evt->data_len);
                }
                output_len += evt->data_len;
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            if (output_buffer != NULL) {
                output_buffer[output_len] = '\0';
                ESP_LOGI(TAG, "HTTP response: %s", output_buffer);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        default: break;
    }
    return ESP_OK;
}

esp_err_t pi_communicator_init(const char* pi_endpoint_url) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            /* All other fields are zero-initialized by default */
        },
    };
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASSWORD);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi initialization finished. Waiting for connection...");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED WIFI EVENT");
        return ESP_FAIL;
    }

    if (s_pi_endpoint_url != NULL) free(s_pi_endpoint_url);
    s_pi_endpoint_url = strdup(pi_endpoint_url);
    if (s_pi_endpoint_url == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for PI endpoint URL.");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "PI Communicator initialized. Target URL: %s", s_pi_endpoint_url);
    return ESP_OK;
}

esp_err_t pi_communicator_send_uid(const RFID_Uid_t* uid) {
    if (uid == NULL || s_pi_endpoint_url == NULL) return ESP_ERR_INVALID_ARG;

	char uid_hex_str[16 * 2 + 1]; // Use a static size of 16 bytes for the UID buffer
    for (int i = 0; i < uid->uid_len; i++) {
        sprintf(&uid_hex_str[i * 2], "%02X", uid->uid[i]);
    }
    uid_hex_str[uid->uid_len * 2] = '\0';

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return ESP_FAIL;
    cJSON_AddStringToObject(root, "uid", uid_hex_str);
    cJSON_AddNumberToObject(root, "timestamp", xTaskGetTickCount() * portTICK_PERIOD_MS / 1000);

    char *json_string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json_string == NULL) return ESP_FAIL;

    esp_http_client_config_t config = {
        .url = s_pi_endpoint_url,
        .event_handler = _http_event_handler,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        free(json_string);
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_string, strlen(json_string));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d, Content_length = %lld",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    free(json_string);
    return err;
}