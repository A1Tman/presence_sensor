#include "ld2420.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_idf_version.h"

#include "driver/gpio.h"
#include "driver/uart.h"

static const char *TAG = "ld2420";

/* =========================
 *   Static state
 * ========================= */
static ld2420_cfg_t s_cfg;
static TaskHandle_t s_task = NULL;

static volatile bool     s_present = false;
static volatile int      s_last_distance_mm = -1;
static volatile int64_t  s_last_distance_us = 0;

static volatile bool     s_uart_active = false;
static volatile int      s_uart_baud = 0;
static volatile int64_t  s_uart_last_byte_us = 0;
static bool              s_warned_no_uart = false;

/* OT2 debounce/hold */
static bool     s_ot2_last = false;     // last *active* sample
static bool     s_ot2_stable = false;   // debounced active state
static int64_t  s_ot2_edge_us = 0;      // last change timestamp
static int64_t  s_last_high_us = 0;     // last time active=true

/* Publishing cadence limiter */
static int64_t  s_last_pub_us = 0;

/* =========================
 *   Utilities
 * ========================= */
static inline int64_t now_us(void) { return esp_timer_get_time(); }
static inline bool pin_is_valid(gpio_num_t pin) { return (pin >= 0) && (pin < GPIO_NUM_MAX); }

/* =========================
 *   UART helpers
 * ========================= */
static esp_err_t uart_setup(int baud)
{
    const uart_config_t uc = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION_MAJOR >= 5
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };
    ESP_RETURN_ON_ERROR(uart_param_config(s_cfg.uart_num, &uc), TAG, "uart_param_config");
    ESP_RETURN_ON_ERROR(uart_set_pin(s_cfg.uart_num, s_cfg.uart_tx, s_cfg.uart_rx,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin");

    if (pin_is_valid(s_cfg.uart_rx)) {
        gpio_pullup_en(s_cfg.uart_rx);  // define RX idle
    }
    return ESP_OK;
}

static bool uart_seems_alive(int check_ms)
{
    const int64_t end_us = now_us() + (int64_t)check_ms * 1000;
    uint8_t tmp[128];
    size_t total = 0;
    while (now_us() < end_us) {
        int n = uart_read_bytes(s_cfg.uart_num, tmp, sizeof(tmp), pdMS_TO_TICKS(10));
        if (n > 0) {
            total += n;
            s_uart_last_byte_us = now_us();
            if (total >= 16) break;
        }
    }
    return total >= 8;
}

static void uart_autobaud(void)
{
    s_uart_active = false;
    s_uart_baud = 0;

    if (uart_setup(s_cfg.baud_primary) == ESP_OK) {
        if (uart_seems_alive(250)) {
            s_uart_active = true;
            s_uart_baud = s_cfg.baud_primary;
            s_warned_no_uart = false;
            ESP_LOGI(TAG, "UART active at %d baud", s_uart_baud);
            return;
        }
    }

    if (uart_setup(s_cfg.baud_fallback) == ESP_OK) {
        if (uart_seems_alive(350)) {
            s_uart_active = true;
            s_uart_baud = s_cfg.baud_fallback;
            s_warned_no_uart = false;
            ESP_LOGI(TAG, "UART active at %d baud (fallback)", s_uart_baud);
            return;
        }
    }
}

/* =========================
 *   Presence + publishing
 * ========================= */
static void publish(bool force, bool present_now)
{
    const int64_t t = now_us();
    const int64_t min_period = (int64_t)s_cfg.min_pub_interval_ms * 1000;

    if (!force && (t - s_last_pub_us) < min_period) return;

    s_last_pub_us = t;
    if (s_cfg.cb) {
        int d = s_last_distance_mm;
        if (d >= 0 && (t - s_last_distance_us) > 2000000LL) d = -1; // distance stale > 2s
        s_cfg.cb(present_now, d);
    }
}

static void update_presence_from_ot2(bool active, int raw_lvl, int64_t t)
{
    if (active != s_ot2_last) {
        s_ot2_last = active;
        s_ot2_edge_us = t;
    }

    const bool stable = (t - s_ot2_edge_us) >= (int64_t)s_cfg.debounce_ms * 1000;
    if (stable) s_ot2_stable = s_ot2_last;

    if (s_ot2_stable) s_last_high_us = t;

    bool new_present = s_present;
    if (s_ot2_stable) {
        new_present = true;
    } else if ((t - s_last_high_us) > (int64_t)s_cfg.hold_on_ms * 1000) {
        new_present = false;
    }

    if (new_present != s_present) {
        s_present = new_present;
        ESP_LOGI(TAG, "Presence %s (lvl=%d, active=%d, polarity=%s)",
                 s_present ? "ON" : "OFF",
                 raw_lvl, active ? 1 : 0,
                 s_cfg.ot2_active_high ? "AH" : "AL");
        publish(true, s_present);
    }
}

/* =========================
 *   ASCII-ish UART scraper
 * ========================= */
static void parse_ascii_blob(const uint8_t *buf, int len, int64_t t_now)
{
    if (!pin_is_valid(s_cfg.ot2_gpio)) {
        for (int i = 0; i < len; ++i) {
            if (i + 1 < len && buf[i] == 'O' && buf[i+1] == 'N') {
                if (!s_present) { s_present = true; publish(true, s_present); } else publish(false, s_present);
            } else if (i + 2 < len && buf[i] == 'O' && buf[i+1] == 'F' && buf[i+2] == 'F') {
                if (s_present) { s_present = false; publish(true, s_present); } else publish(false, s_present);
            }
        }
    }

    int last_mm = -1;
    for (int i = 0; i < len; ) {
        if (isdigit((int)buf[i])) {
            int j = i, val = 0, digits = 0;
            while (j < len && isdigit((int)buf[j]) && digits < 6) {
                val = val * 10 + (buf[j] - '0');
                j++; digits++;
            }
            if (val >= 5 && val <= 6000) last_mm = val;
            i = j;
        } else {
            i++;
        }
    }

    if (last_mm >= 0) {
        s_last_distance_mm = last_mm;
        s_last_distance_us = t_now;
        publish(false, s_present);
    }
}

/* =========================
 *   Worker task
 * ========================= */
static void ld2420_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LD2420 task");

    /* UART driver install */
    const int rx_buf = 2048;
    const int tx_buf = 0;
    ESP_ERROR_CHECK(uart_driver_install(s_cfg.uart_num, rx_buf, tx_buf, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_setup(s_cfg.baud_primary));
    s_uart_last_byte_us = now_us();
    uart_autobaud();

    /* OT2 setup (polled; no interrupts) */
    if (pin_is_valid(s_cfg.ot2_gpio)) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << s_cfg.ot2_gpio),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io));
        gpio_intr_disable(s_cfg.ot2_gpio);
        gpio_set_intr_type(s_cfg.ot2_gpio, GPIO_INTR_DISABLE);
    }

    /* Initial state & publish */
    s_last_pub_us = 0;
    s_last_distance_mm = -1;
    s_last_distance_us = 0;
    s_last_high_us = 0;
    s_ot2_last = false;
    s_ot2_stable = false;

    if (pin_is_valid(s_cfg.ot2_gpio)) {
        int idle = gpio_get_level(s_cfg.ot2_gpio);

        // Infer polarity from idle: idle HIGH => active-low; idle LOW => active-high.
        bool inferred_active_high = (idle == 0);
        if (s_cfg.ot2_active_high != inferred_active_high) {
            ESP_LOGW(TAG, "OT2 polarity adjusted: configured %s → inferred %s (idle=%d)",
                     s_cfg.ot2_active_high ? "active-high" : "active-low",
                     inferred_active_high ? "active-high" : "active-low",
                     idle);
            s_cfg.ot2_active_high = inferred_active_high;
        } else {
            ESP_LOGI(TAG, "OT2 polarity: %s (idle=%d)",
                     s_cfg.ot2_active_high ? "active-high" : "active-low", idle);
        }

        // Now set pulls to match polarity
        if (s_cfg.ot2_active_high) {
            // idle LOW expected → help it with pulldown
            gpio_pullup_dis(s_cfg.ot2_gpio);
            gpio_pulldown_en(s_cfg.ot2_gpio);
        } else {
            // idle HIGH expected → help it with pullup
            gpio_pulldown_dis(s_cfg.ot2_gpio);
            gpio_pullup_en(s_cfg.ot2_gpio);
        }

        bool active = s_cfg.ot2_active_high ? (idle == 1) : (idle == 0);
        const int64_t t = now_us();
        s_ot2_last = active;
        s_ot2_edge_us = t - (int64_t)s_cfg.debounce_ms * 1000;
        s_ot2_stable = active;
        s_present = active;
        if (active) s_last_high_us = t;

        ESP_LOGI(TAG, "Initial presence: %s (lvl=%d, active=%d)",
                 s_present ? "ON" : "OFF", idle, active ? 1 : 0);
        publish(true, s_present);
    } else {
        s_present = false;
        ESP_LOGI(TAG, "OT2 not wired; starting with presence OFF");
        publish(true, s_present);
    }

    /* Loop */
    const TickType_t tick = pdMS_TO_TICKS(s_cfg.sample_ms);
    uint8_t buf[512];
    int64_t last_auto_us = 0;

    while (1) {
        vTaskDelay(tick);
        const int64_t tnow = now_us();

        /* Poll OT2 */
        if (pin_is_valid(s_cfg.ot2_gpio)) {
            int lvl = gpio_get_level(s_cfg.ot2_gpio);
            bool active = s_cfg.ot2_active_high ? (lvl == 1) : (lvl == 0);
            update_presence_from_ot2(active, lvl, tnow);
        }

        /* UART scraping */
        if (s_uart_active) {
            int n = uart_read_bytes(s_cfg.uart_num, buf, sizeof(buf), 0);
            if (n > 0) {
                s_uart_last_byte_us = tnow;
                parse_ascii_blob(buf, n, tnow);
                s_warned_no_uart = false;
            } else {
                if ((tnow - s_uart_last_byte_us) > 15000000LL) { // 15s idle
                    s_uart_active = false;
                }
            }
        }

        /* Retry autobaud every ~5s if inactive; warn once until traffic returns */
        if (!s_uart_active && (tnow - last_auto_us) > 5000000LL) {
            last_auto_us = tnow;
            uart_autobaud();
            if (!s_uart_active && !s_warned_no_uart) {
                ESP_LOGW(TAG, "No UART traffic detected at %d or %d baud",
                         s_cfg.baud_primary, s_cfg.baud_fallback);
                s_warned_no_uart = true;
            }
        }

        /* Periodic heartbeat publish (rate-limited inside) */
        publish(false, s_present);
    }
}

/* =========================
 *   Public API
 * ========================= */
esp_err_t ld2420_init(const ld2420_cfg_t *cfg)
{
    if (!cfg || !cfg->cb) return ESP_ERR_INVALID_ARG;

    s_cfg = *cfg;

    if (s_cfg.sample_ms <= 0)            s_cfg.sample_ms = 50;
    if (s_cfg.debounce_ms <= 0)          s_cfg.debounce_ms = 150;
    if (s_cfg.hold_on_ms <= 0)           s_cfg.hold_on_ms = 3000;
    if (s_cfg.min_pub_interval_ms <= 0)  s_cfg.min_pub_interval_ms = 10000;

    if (s_cfg.baud_primary   <= 0) s_cfg.baud_primary   = 115200;  // FW ≥ 1.5.3
    if (s_cfg.baud_fallback  <= 0) s_cfg.baud_fallback  = 256000;  // legacy

    if (!pin_is_valid(s_cfg.uart_tx) || !pin_is_valid(s_cfg.uart_rx)) {
        ESP_LOGE(TAG, "Invalid UART pins");
        return ESP_ERR_INVALID_ARG;
    }

    s_present = false;
    s_last_distance_mm = -1;
    s_last_distance_us = 0;

    s_uart_active = false;
    s_uart_baud = 0;
    s_uart_last_byte_us = 0;
    s_warned_no_uart = false;

    return ESP_OK;
}

esp_err_t ld2420_start(void)
{
    if (s_task) return ESP_OK;
    BaseType_t ok = xTaskCreate(ld2420_task, "ld2420_task", 4096, NULL, 4, &s_task);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

void ld2420_stop(void)
{
    if (!s_task) return;
    vTaskDelete(s_task);
    s_task = NULL;

    uart_driver_delete(s_cfg.uart_num);
}

bool ld2420_get_present(void)          { return s_present; }
int  ld2420_get_last_distance_mm(void) { return s_last_distance_mm; }
bool ld2420_uart_active(void)          { return s_uart_active; }
int  ld2420_active_baud(void)          { return s_uart_baud; }
