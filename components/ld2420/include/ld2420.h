#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef GPIO_NUM_NC
#define GPIO_NUM_NC ((gpio_num_t)-1)  // Not connected
#endif

// Presence callback: distance_mm == -1 when not available
typedef void (*ld2420_presence_cb_t)(bool present, int distance_mm);

typedef struct {
    // UART wiring (LD2420 TX -> ESP RX, LD2420 RX <- ESP TX)
    uart_port_t uart_num;       // e.g. UART_NUM_0 on ESP32-C3
    gpio_num_t  uart_tx;        // ESP TX pin to LD2420 RX (e.g. GPIO21)
    gpio_num_t  uart_rx;        // ESP RX pin from LD2420 TX/OT1 (e.g. GPIO20)
    int         baud_primary;   // FW ≥1.5.3: 115200
    int         baud_fallback;  // Legacy FW: 256000

    // Digital presence pin (OT2)
    gpio_num_t  ot2_gpio;       // e.g. GPIO4 (or GPIO_NUM_NC if unused)
    bool        ot2_active_high;// true for typical LD2420 boards

    // Sampling / smoothing
    int         sample_ms;      // task loop period (e.g. 50)
    int         debounce_ms;    // debounce for OT2 edges (e.g. 150)
    int         hold_on_ms;     // keep presence true after low (e.g. 3000)

    // Publishing policy
    int         min_pub_interval_ms;  // force publish at least every N ms (e.g. 10000)
    int         stale_after_ms;       // consider distance stale after N ms (e.g. 2000)

    // App callback
    ld2420_presence_cb_t cb;

    // Optional: boot/probe TX sequences to enable streaming on some LD2420 variants
    // Provide either or both. Strings are sent as-is (ASCII) and/or parsed as
    // space-separated hex bytes (e.g., "AA 55 01 00").
    const char *boot_tx_ascii;     // NULL or empty to disable
    const char *boot_tx_hex;       // NULL or empty to disable
    int         boot_tx_repeat;    // how many times to send per probe (e.g., 1-3)
    int         boot_tx_delay_ms;  // delay between repeats (e.g., 20-50ms)
    
    // UART probing strategy
    // Set fixed-baud mode to stop repeated autobaud scanning/log churn on modules
    // that don’t emit stream data and only need UART for configuration.
    bool        uart_fixed_baud;   // true = stick to fixed_baud_rate (no periodic scanning)
    int         fixed_baud_rate;   // if <=0, uses baud_primary
} ld2420_cfg_t;

esp_err_t ld2420_init(const ld2420_cfg_t *cfg);
esp_err_t ld2420_start(void);
void      ld2420_stop(void);

// Helpers
bool ld2420_get_present(void);
int  ld2420_get_last_distance_mm(void);
bool ld2420_uart_active(void);       // true if we detected a live UART stream
int  ld2420_active_baud(void);       // returns 0 if unknown

// Runtime tuning
void ld2420_set_debounce_ms(int v);
void ld2420_set_hold_on_ms(int v);
int  ld2420_get_debounce_ms(void);
int  ld2420_get_hold_on_ms(void);

// MQTT command handler (text commands routed from HA)
void ld2420_mqtt_cmd(const char *payload, int len);

#ifdef __cplusplus
}
#endif
