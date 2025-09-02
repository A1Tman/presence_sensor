#include "ld2420_simple.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_idf_version.h"

static const char *TAG = "ld2420_simple";

static ld2420_simple_cfg_t s_cfg;
static TaskHandle_t s_task = NULL;
static volatile bool    s_present = false;
static volatile int     s_last_mm = -1;

static inline bool pin_is_valid(gpio_num_t pin) { return (pin >= 0) && (pin < GPIO_NUM_MAX); }

static void send_boot_seq(void)
{
    const char *ascii = s_cfg.boot_tx_ascii;
    const char *hex   = s_cfg.boot_tx_hex;
    int reps = s_cfg.boot_tx_repeat > 0 ? s_cfg.boot_tx_repeat : 2;
    int dly  = s_cfg.boot_tx_delay_ms > 0 ? s_cfg.boot_tx_delay_ms : 20;

    for (int r = 0; r < reps; ++r) {
        if (ascii && ascii[0]) {
            uart_write_bytes(s_cfg.uart_num, ascii, strlen(ascii));
        }
        if (hex && hex[0]) {
            const char *s = hex;
            while (*s) {
                while (*s == ' ' || *s == ',' || *s == '\t') s++;
                if (!*s) break;
                char *end = NULL;
                long v = strtol(s, &end, 16);
                if (end == s) break;
                uint8_t b = (uint8_t)(v & 0xFF);
                uart_write_bytes(s_cfg.uart_num, &b, 1);
                s = end;
            }
        }
        if (dly > 0) vTaskDelay(pdMS_TO_TICKS(dly));
    }
    uart_wait_tx_done(s_cfg.uart_num, pdMS_TO_TICKS(50));
}

static void flush_line(const char *line, int len)
{
    if (!line || len <= 0) return;
    while (len > 0 && (line[len-1] == '\r' || line[len-1] == '\n' || isspace((int)line[len-1]))) len--;
    if (len <= 0) return;
    if (len >= 6 && !strncmp(line, "Range ", 6)) {
        char *end = NULL;
        long cm = strtol(line + 6, &end, 10);
        if (end != line + 6 && cm >= 0 && cm <= 600) {
            int mm = (int)cm * 10;
            bool present = (cm > 0);
            bool changed = (present != s_present) || (present && mm != s_last_mm);
            s_present = present;
            s_last_mm = mm;
            if (changed && s_cfg.cb) {
                s_cfg.cb(present, present ? mm : -1);
            }
        }
    }
}

static void parse_chunk(const uint8_t *buf, int len)
{
    static char line[128];
    static int L = 0;
    for (int i = 0; i < len; ++i) {
        char c = (char)buf[i];
        if (c == '\n' || c == '\r') {
            if (L > 0) {
                flush_line(line, L);
                L = 0;
            }
        } else if (L < (int)sizeof(line) - 1) {
            line[L++] = c;
        }
    }
}

static void task_fn(void *arg)
{
    ESP_LOGI(TAG, "Starting LD2420 simple task @%d baud", s_cfg.baud);

    // UART setup
    const uart_config_t uc = {
        .baud_rate  = s_cfg.baud > 0 ? s_cfg.baud : 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION_MAJOR >= 5
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };
    ESP_ERROR_CHECK(uart_param_config(s_cfg.uart_num, &uc));
    ESP_ERROR_CHECK(uart_set_pin(s_cfg.uart_num, s_cfg.uart_tx, s_cfg.uart_rx,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    if (pin_is_valid(s_cfg.uart_rx)) {
        gpio_pullup_en(s_cfg.uart_rx);
    }

    // Install driver with simple RX buffer first
    const int rx_buf = 2048;
    ESP_ERROR_CHECK(uart_driver_install(s_cfg.uart_num, rx_buf, 0, 0, NULL, 0));
    uart_flush_input(s_cfg.uart_num);
    // Small RX threshold so we get data quickly
    uart_set_rx_full_threshold(s_cfg.uart_num, 1);
    uart_set_rx_timeout(s_cfg.uart_num, 2);

    // Optional boot sequence
    if ((s_cfg.boot_tx_ascii && s_cfg.boot_tx_ascii[0]) || (s_cfg.boot_tx_hex && s_cfg.boot_tx_hex[0])) {
        send_boot_seq();
    } else {
        // Built-in nudge: INIT frame and ASCII report toggle
        ld2420_simple_cfg_t tmp = s_cfg;
        tmp.boot_tx_ascii = "UARTRPT=1\r\n";
        tmp.boot_tx_hex   = "FD FC FB FA 08 00 12 00 00 00 64 00 00 00 04 03 02 01";
        tmp.boot_tx_repeat= 2; tmp.boot_tx_delay_ms = 20;
        s_cfg = tmp; send_boot_seq(); s_cfg = tmp; // keep values around
    }

    // Initial publish
    if (s_cfg.cb) s_cfg.cb(false, -1);

    const TickType_t tick = pdMS_TO_TICKS(s_cfg.sample_ms > 0 ? s_cfg.sample_ms : 50);
    uint8_t buf[512];

    while (1) {
        vTaskDelay(tick);

        int n = uart_read_bytes(s_cfg.uart_num, buf, sizeof(buf), 0);
        if (n > 0) {
            int m = n < 16 ? n : 16;
            char hex[3*16+1]; int k = 0;
            for (int i = 0; i < m; i++) k += snprintf(hex + k, sizeof(hex) - k, "%02X ", buf[i]);
            ESP_LOGD(TAG, "UART rx %dB: %s%s", n, hex, (n > m ? "..." : ""));
            parse_chunk(buf, n);
        }
    }
}

esp_err_t ld2420_simple_init(const ld2420_simple_cfg_t *cfg)
{
    if (!cfg || !cfg->cb) return ESP_ERR_INVALID_ARG;
    s_cfg = *cfg;
    s_present = false;
    s_last_mm = -1;
    return ESP_OK;
}

esp_err_t ld2420_simple_start(void)
{
    if (s_task) return ESP_OK;
    BaseType_t ok = xTaskCreate(task_fn, "ld2420_simple", 3072, NULL, 4, &s_task);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

void ld2420_simple_stop(void)
{
    if (!s_task) return;
    vTaskDelete(s_task);
    s_task = NULL;
    uart_driver_delete(s_cfg.uart_num);
}

bool ld2420_simple_get_present(void) { return s_present; }
int  ld2420_simple_get_last_distance_mm(void) { return s_last_mm; }
