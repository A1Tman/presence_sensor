#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "ha_mqtt.h"

// Import secure configuration early for build-time toggles
#include "secrets.h"

#if USE_LD2411
#include "ld2411.h"
#elif USE_LD2420_SIMPLE
#include "ld2420_simple.h"
#else
#include "ld2420.h"   // LD2420 default
#endif

// Optional: allow overriding OT2 polarity from config
#ifndef LD2420_OT2_ACTIVE_HIGH
#define LD2420_OT2_ACTIVE_HIGH 0
#endif

// ==================== COMPILE-TIME CHECKS ====================
#ifndef SECRETS_H
#error "secrets.h not found! Copy secrets.h.template to secrets.h and configure your credentials"
#endif

#if !defined(WIFI_SSID) || !defined(WIFI_PASSWORD)
#error "WiFi credentials not configured in secrets.h"
#endif

#if !defined(DEVICE_ID) || !defined(DEVICE_NAME)
#error "Device identification not configured in secrets.h"
#endif

// ==================== DEVICE VERSION & BUILD INFO ====================
#define DEVICE_VERSION "1.0.0"
#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__

#if MQTT_USE_TLS
#define MQTT_URI_PREFIX "mqtts://"
#else
#define MQTT_URI_PREFIX "mqtt://"
#endif

// Task Configuration
#define MAIN_TASK_STACK_SIZE    4096
#define MAIN_TASK_PRIORITY      5

// ==================== GLOBALS ====================
static const char *TAG = "PRESENCE_SENSOR";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

// WiFi event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// ==================== SECURITY / CONFIG DIAGNOSTICS ====================
static void log_security_info(void)
{
    ESP_LOGI(TAG, "=== Security Configuration ===");
    ESP_LOGI(TAG, "WiFi SSID: %.3s*** (hidden)", WIFI_SSID);
    ESP_LOGI(TAG, "MQTT Broker: %s:%d", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    ESP_LOGI(TAG, "MQTT Auth: %s", (strlen(MQTT_USERNAME) > 0) ? "Enabled" : "Disabled");
    ESP_LOGI(TAG, "TLS/SSL: %s", MQTT_USE_TLS ? "Enabled" : "Disabled");
    ESP_LOGI(TAG, "Debug Web Interface: %s", ENABLE_WEB_INTERFACE ? "Enabled" : "Disabled");
}

static void validate_configuration(void)
{
    bool config_valid = true;

    if (strlen(WIFI_SSID) == 0 || strlen(WIFI_PASSWORD) == 0) {
        ESP_LOGE(TAG, "Invalid WiFi credentials in secrets.h");
        config_valid = false;
    }
    if (strlen(MQTT_BROKER_HOST) == 0) {
        ESP_LOGE(TAG, "MQTT broker host not configured in secrets.h");
        config_valid = false;
    }
    if (strlen(DEVICE_ID) == 0 || strlen(DEVICE_NAME) == 0) {
        ESP_LOGE(TAG, "Device identification not configured in secrets.h");
        config_valid = false;
    }
    if (strcmp(WIFI_SSID, "YourWiFiNetworkName") == 0) {
        ESP_LOGE(TAG, "Please update WIFI_SSID in secrets.h - still using template values!");
        config_valid = false;
    }
    if (strcmp(MQTT_BROKER_HOST, "192.168.1.100") == 0) {
        ESP_LOGW(TAG, "Using default MQTT broker IP - make sure this is correct for your network");
    }

    if (!config_valid) {
        ESP_LOGE(TAG, "Configuration validation failed! Please check secrets.h");
        ESP_LOGE(TAG, "Copy secrets.h.template to secrets.h and configure your settings");
        esp_restart();
    }

    ESP_LOGI(TAG, "Configuration validation passed ✓");
}

// ==================== MQTT STARTER (after Wi-Fi IP) ====================
static void start_mqtt_after_wifi(void) {
    char uri[96];
    int port = MQTT_USE_TLS ? MQTT_TLS_PORT : MQTT_BROKER_PORT;
    snprintf(uri, sizeof(uri), "%s%s:%d", MQTT_URI_PREFIX, MQTT_BROKER_HOST, port);

    static char s_device_model[64];
#if USE_LD2411
    snprintf(s_device_model, sizeof(s_device_model), "HLK-LD2411 + %s", CONFIG_IDF_TARGET);
#elif USE_LD2420_SIMPLE
    snprintf(s_device_model, sizeof(s_device_model), "HLK-LD2420 (simple) + %s", CONFIG_IDF_TARGET);
#else
    snprintf(s_device_model, sizeof(s_device_model), "HLK-LD2420 + %s", CONFIG_IDF_TARGET);
#endif

    static const char *s_legacy_ids[] = { DEVICE_ID };
    ha_mqtt_cfg_t cfg = {
        .broker_uri     = uri,                                            // e.g. "mqtt://192.168.1.23:1883"
        .username       = (strlen(MQTT_USERNAME) ? MQTT_USERNAME : NULL), // NULL if empty
        .password       = (strlen(MQTT_PASSWORD) ? MQTT_PASSWORD : NULL),
        .friendly_name  = DEVICE_NAME,
        .suggested_area = DEVICE_LOCATION,
        .app_version    = DEVICE_VERSION,
        .discovery_prefix = HA_DISCOVERY_PREFIX,
        .device_model   = s_device_model,
        // Provide prior node IDs to clean up retained HA discovery topics
        .legacy_node_ids = s_legacy_ids,
        .legacy_node_ids_count = (int)(sizeof(s_legacy_ids)/sizeof(s_legacy_ids[0])),
#if USE_LD2411
        .distance_supported = false,
#else
        // LD2420 spec: serial protocol only; no distance on output pins.
        // Disable HA distance entity by default to reflect that many boards
        // don’t expose reliable distance, unless explicitly enabled later.
        .distance_supported = false,
#endif
        .get_debounce_ms =
#if USE_LD2411
            ld2411_get_debounce_ms,
#elif USE_LD2420_SIMPLE
            NULL,
#else
            ld2420_get_debounce_ms,
#endif
        .get_hold_on_ms =
#if USE_LD2411
            ld2411_get_hold_on_ms,
#elif USE_LD2420_SIMPLE
            NULL,
#else
            ld2420_get_hold_on_ms,
#endif
        .set_debounce_ms =
#if USE_LD2411
            ld2411_set_debounce_ms,
#elif USE_LD2420_SIMPLE
            NULL,
#else
            ld2420_set_debounce_ms,
#endif
        .set_hold_on_ms =
#if USE_LD2411
            ld2411_set_hold_on_ms,
#elif USE_LD2420_SIMPLE
            NULL,
#else
            ld2420_set_hold_on_ms,
#endif
        .get_out_active_high =
#if USE_LD2411
            ld2411_get_out_active_high,
#else
            NULL,
#endif
        .set_out_active_high =
#if USE_LD2411
            ld2411_set_out_active_high,
#else
            NULL,
#endif
        .get_out_pullup_enabled =
#if USE_LD2411
            ld2411_get_out_pullup_enabled,
#else
            NULL,
#endif
        .set_out_pullup_enabled =
#if USE_LD2411
            ld2411_set_out_pullup_enabled,
#else
            NULL,
#endif
        .get_uart_presence_enabled =
#if USE_LD2411
            ld2411_get_uart_presence_enabled,
#else
            NULL,
#endif
        .set_uart_presence_enabled =
#if USE_LD2411
            ld2411_set_uart_presence_enabled,
#else
            NULL,
#endif
        .get_presence_source =
#if USE_LD2411
            ld2411_get_presence_source,
#else
            NULL,
#endif
        .set_presence_source =
#if USE_LD2411
            ld2411_set_presence_source,
#else
            NULL,
#endif
        .get_distance_thresh_cm =
#if USE_LD2411
            ld2411_get_distance_thresh_cm,
#else
            NULL,
#endif
        .set_distance_thresh_cm =
#if USE_LD2411
            ld2411_set_distance_thresh_cm,
#else
            NULL,
#endif
        .get_distance_presence_enable =
#if USE_LD2411
            ld2411_get_distance_presence_enable,
#else
            NULL,
#endif
        .set_distance_presence_enable =
#if USE_LD2411
            ld2411_set_distance_presence_enable,
#else
            NULL,
#endif
        .action_force_publish =
#if USE_LD2411
            ld2411_action_force_publish,
#else
            NULL,
#endif
        .action_reautobaud =
#if USE_LD2411
            ld2411_action_reautobaud,
#else
            NULL,
#endif
        .action_reset_tuning =
#if USE_LD2411
            ld2411_action_reset_tuning,
#else
            NULL,
#endif
        .ld2420_handle_cmd =
#if USE_LD2411
            NULL,
#elif USE_LD2420_SIMPLE
            NULL,
#else
            ld2420_mqtt_cmd,
#endif
    };
    ha_mqtt_init(&cfg);
    ha_mqtt_start();
}

// ==================== Wi-Fi EVENT HANDLER ====================
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            ESP_LOGI(TAG, "WiFi station started, attempting connection...");
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            if (s_retry_num < WIFI_MAXIMUM_RETRY) {
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGW(TAG, "WiFi connection failed, retrying... (%d/%d)", s_retry_num, WIFI_MAXIMUM_RETRY);
            } else {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGE(TAG, "WiFi connection failed after %d attempts", WIFI_MAXIMUM_RETRY);
            }
            // Do NOT stop MQTT here; let it auto-reconnect and keep LWT semantics.
            break;

        default:
            break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "WiFi connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
        ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        // Bring up MQTT (publishes HA discovery + availability)
        start_mqtt_after_wifi();
    }
}


// ==================== Wi-Fi INITIALIZATION ====================
static esp_err_t wifi_init_sta(void)
{
    ESP_LOGI(TAG, "Initializing WiFi...");

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .scan_method = WIFI_SCAN_METHOD,
            .sort_method = WIFI_SORT_METHOD,
            .threshold.rssi = WIFI_RSSI_THRESHOLD,
            .threshold.authmode = WIFI_AUTH_MODE_THRESHOLD,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Improve link stability on weak RSSI: disable power save
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "WiFi configuration completed. Connecting to network...");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to WiFi network successfully ✓");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to WiFi network ✗");
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Unexpected WiFi event");
        return ESP_FAIL;
    }
}

// ==================== NVS INITIALIZATION ====================
static esp_err_t nvs_init(void)
{
    ESP_LOGI(TAG, "Initializing NVS storage...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "NVS storage initialized successfully ✓");
    }

    return ret;
}

// ==================== DEVICE INFO ====================
static void print_device_info(void)
{
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "Device: %s (%s)", DEVICE_NAME, DEVICE_ID);
    ESP_LOGI(TAG, "Version: %s", DEVICE_VERSION);
    ESP_LOGI(TAG, "Built: %s %s", BUILD_DATE, BUILD_TIME);
    ESP_LOGI(TAG, "Location: %s", DEVICE_LOCATION);
    ESP_LOGI(TAG, "====================================");

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Hardware: %s with %d CPU core(s)", CONFIG_IDF_TARGET, chip_info.cores);
    ESP_LOGI(TAG, "Features: WiFi%s%s",
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGI(TAG, "Memory: %lu KB free, %lu KB minimum",
             (unsigned long)(esp_get_free_heap_size() / 1024),
             (unsigned long)(esp_get_minimum_free_heap_size() / 1024));

    uint32_t flash_size;
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        ESP_LOGI(TAG, "Flash: %lu MB total", (unsigned long)(flash_size / (1024 * 1024)));
    }

    ESP_LOGI(TAG, "====================================");
}

// ==================== STATUS MONITORING ====================
static void status_led_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Status monitoring task started");

    uint32_t tick = 0;
    while (1) {
        uint32_t free_heap = esp_get_free_heap_size();

        ESP_LOGD(TAG, "System OK - Free heap: %lu KB", (unsigned long)(free_heap / 1024));

        if (free_heap < 50000) {  // Less than ~50 KB
            ESP_LOGW(TAG, "Low memory warning: %lu bytes free", (unsigned long)free_heap);
        }

        // Piggyback MQTT diagnostics (~60s cadence inside ha_mqtt)
        if ((tick++ % 6) == 0) {  // task runs every 10s → call ~each minute
            ha_mqtt_publish_rssi_now(); // no-op if MQTT not connected
        }

        // TODO: LED blink / OLED status

        vTaskDelay(pdMS_TO_TICKS(10000)); // Every 10 seconds
    }
}

// ==================== MAIN TASK ====================
static void main_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Main application task started");

    validate_configuration();
    ESP_ERROR_CHECK(nvs_init());

    // Wi-Fi first
    esp_err_t wifi_ret = wifi_init_sta();
    if (wifi_ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi initialization failed, continuing without network...");
    }

    print_device_info();
    log_security_info();

    // Background status task
    xTaskCreate(status_led_task, "status_monitor", 2048, NULL, 1, NULL);

    // --- Radar sensor start (LD2420 default, LD24211 if enabled) ---
    // Allow overrides from config for debounce/hold
    int cfg_debounce_ms = 80;
    int cfg_hold_on_ms  = 1000;
#ifdef LD2420_DEBOUNCE_MS
    cfg_debounce_ms = LD2420_DEBOUNCE_MS;
#endif
#ifdef PRESENCE_TIMEOUT_SECONDS
    cfg_hold_on_ms  = (PRESENCE_TIMEOUT_SECONDS) * 1000;
#endif
#ifdef LD2420_HOLD_ON_MS
    cfg_hold_on_ms  = LD2420_HOLD_ON_MS;
#endif
    // NOTE: Keep console on USB-CDC or a different UART to avoid conflicts with sensor UART.
#if USE_LD2411
    ld2411_cfg_t lcfg = {
        .uart_num       = (uart_port_t)LD2420_UART_NUM,
        .uart_tx        = (gpio_num_t)LD2420_UART_TX_PIN,
        .uart_rx        = (gpio_num_t)LD2420_UART_RX_PIN,
        .baud_primary   = LD2420_BAUD_RATE,
        .baud_fallback  = 256000,

        .ot2_gpio       = USE_LD2420_OT2 ? (gpio_num_t)LD2420_OT2_PIN : GPIO_NUM_NC,
        .ot2_active_high= (LD2420_OT2_ACTIVE_HIGH ? true : false),

        .sample_ms      = 50,
        .debounce_ms    = cfg_debounce_ms,
        .hold_on_ms     = cfg_hold_on_ms,

        .min_pub_interval_ms = 10000,
        .stale_after_ms = 2000,
        .cb = ha_mqtt_publish_presence,
    };
#ifdef LD2420_BOOT_TX_ASCII
    lcfg.boot_tx_ascii = LD2420_BOOT_TX_ASCII;
#endif
#ifdef LD2420_BOOT_TX_HEX
    lcfg.boot_tx_hex = LD2420_BOOT_TX_HEX;
#endif
#ifdef LD2420_BOOT_TX_REPEAT
    lcfg.boot_tx_repeat = LD2420_BOOT_TX_REPEAT;
#endif
#ifdef LD2420_BOOT_TX_DELAY_MS
    lcfg.boot_tx_delay_ms = LD2420_BOOT_TX_DELAY_MS;
#endif
    ESP_ERROR_CHECK(ld2411_init(&lcfg));
    ESP_ERROR_CHECK(ld2411_start());
#else
#if USE_LD2420_SIMPLE
    ld2420_simple_cfg_t lcfg = {
        .uart_num       = (uart_port_t)LD2420_UART_NUM,
        .uart_tx        = (gpio_num_t)LD2420_UART_TX_PIN,
        .uart_rx        = (gpio_num_t)LD2420_UART_RX_PIN,
        .baud           = LD2420_BAUD_RATE,
        .sample_ms      = 50,
        .hold_ms        = cfg_hold_on_ms,
        .ot2_gpio       = USE_LD2420_OT2 ? (gpio_num_t)LD2420_OT2_PIN : GPIO_NUM_NC,
        .ot2_active_high= (LD2420_OT2_ACTIVE_HIGH ? true : false),
        .debounce_ms    = cfg_debounce_ms,
        .cb             = ha_mqtt_publish_presence,
    };
#ifdef LD2420_BOOT_TX_ASCII
    lcfg.boot_tx_ascii = LD2420_BOOT_TX_ASCII;
#endif
#ifdef LD2420_BOOT_TX_HEX
    lcfg.boot_tx_hex = LD2420_BOOT_TX_HEX;
#endif
#ifdef LD2420_BOOT_TX_REPEAT
    lcfg.boot_tx_repeat = LD2420_BOOT_TX_REPEAT;
#endif
#ifdef LD2420_BOOT_TX_DELAY_MS
    lcfg.boot_tx_delay_ms = LD2420_BOOT_TX_DELAY_MS;
#endif
    ESP_ERROR_CHECK(ld2420_simple_init(&lcfg));
    ESP_ERROR_CHECK(ld2420_simple_start());
#else
    ld2420_cfg_t lcfg = {
        .uart_num       = (uart_port_t)LD2420_UART_NUM,  // sensor on UART1 by default
        .uart_tx        = (gpio_num_t)LD2420_UART_TX_PIN,  // ESP TX -> sensor RX
        .uart_rx        = (gpio_num_t)LD2420_UART_RX_PIN,  // ESP RX <- sensor TX/OT1
        .baud_primary   = LD2420_BAUD_RATE,
        .baud_fallback  = 256000,

        .ot2_gpio       = USE_LD2420_OT2 ? (gpio_num_t)LD2420_OT2_PIN : GPIO_NUM_NC,
        .ot2_active_high= (LD2420_OT2_ACTIVE_HIGH ? true : false),

        .sample_ms      = 50,
        .debounce_ms    = cfg_debounce_ms,
        .hold_on_ms     = cfg_hold_on_ms,

        .min_pub_interval_ms = 10000,
        .cb = ha_mqtt_publish_presence,
        // Prefer a fixed baud to avoid constant autobaud scans on non-streaming firmware
        .uart_fixed_baud = true,
        .fixed_baud_rate = LD2420_BAUD_RATE,
    };
#ifdef LD2420_BOOT_TX_ASCII
    lcfg.boot_tx_ascii = LD2420_BOOT_TX_ASCII;
#endif
#ifdef LD2420_BOOT_TX_HEX
    lcfg.boot_tx_hex = LD2420_BOOT_TX_HEX;
#endif
#ifdef LD2420_BOOT_TX_REPEAT
    lcfg.boot_tx_repeat = LD2420_BOOT_TX_REPEAT;
#endif
#ifdef LD2420_BOOT_TX_DELAY_MS
    lcfg.boot_tx_delay_ms = LD2420_BOOT_TX_DELAY_MS;
#endif
    ESP_ERROR_CHECK(ld2420_init(&lcfg));
    ESP_ERROR_CHECK(ld2420_start());
#endif
#endif

    ESP_LOGI(TAG, "✓ Core system initialization completed");
#if USE_LD2411
    ESP_LOGI(TAG, "MQTT & HA integration active; LD2411 running");
#elif USE_LD2420_SIMPLE
    ESP_LOGI(TAG, "MQTT & HA integration active; LD2420 (simple) running");
#else
    ESP_LOGI(TAG, "MQTT & HA integration active; LD2420 running");
#endif

    // Main loop (lightweight; most work in tasks)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000)); // housekeeping tick
    }
}

// ==================== APPLICATION ENTRY POINT ====================
void app_main(void)
{
    // Logging levels
    esp_log_level_set("*", LOG_LEVEL_DEFAULT);
    esp_log_level_set("wifi", LOG_LEVEL_WIFI);
    esp_log_level_set("ld2420", LOG_LEVEL_LD2420);
    esp_log_level_set("ld2411", LOG_LEVEL_LD2420);
    esp_log_level_set("ld2420_simple", LOG_LEVEL_LD2420);
    esp_log_level_set("ha_mqtt", LOG_LEVEL_MQTT);

    ESP_LOGI(TAG, "🚀 Starting %s v%s", DEVICE_NAME, DEVICE_VERSION);
    ESP_LOGI(TAG, "Device ID: %s", DEVICE_ID);

    xTaskCreate(main_task, "main_task", MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "✓ Application started successfully");
}
