#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef GPIO_NUM_NC
#define GPIO_NUM_NC ((gpio_num_t)-1)
#endif

// Presence callback: distance_mm == -1 when not available
typedef void (*ld2411_presence_cb_t)(bool present, int distance_mm);

typedef struct {
    // UART wiring (optional for LD2411; presence works with OUT only)
    uart_port_t uart_num;       // e.g. UART_NUM_1
    gpio_num_t  uart_tx;        // ESP TX pin to sensor RX
    gpio_num_t  uart_rx;        // ESP RX pin from sensor TX/OT1
    int         baud_primary;   // default 115200
    int         baud_fallback;  // e.g., 256000

    // Digital presence pin (OUT or equivalent)
    gpio_num_t  ot2_gpio;       // GPIO_NUM_NC if unused
    bool        ot2_active_high;// true if output is active-high

    // Sampling / smoothing
    int         sample_ms;      // task loop period (e.g. 50)
    int         debounce_ms;    // debounce for OT2 edges
    int         hold_on_ms;     // keep presence true after low

    // Publishing policy
    int         min_pub_interval_ms;  // minimum interval between publishes
    int         stale_after_ms;       // distance staleness (unused if no UART)

    // App callback
    ld2411_presence_cb_t cb;

    // Optional boot/probe sequences if needed later
    const char *boot_tx_ascii;
    const char *boot_tx_hex;
    int         boot_tx_repeat;
    int         boot_tx_delay_ms;
} ld2411_cfg_t;

esp_err_t ld2411_init(const ld2411_cfg_t *cfg);
esp_err_t ld2411_start(void);
void      ld2411_stop(void);

// Helpers
bool ld2411_get_present(void);
int  ld2411_get_last_distance_mm(void);
bool ld2411_uart_active(void);
int  ld2411_active_baud(void);

// Runtime tuning
void ld2411_set_debounce_ms(int v);
void ld2411_set_hold_on_ms(int v);
int  ld2411_get_debounce_ms(void);
int  ld2411_get_hold_on_ms(void);

// Runtime toggles
bool ld2411_get_out_active_high(void);
void ld2411_set_out_active_high(bool on);
bool ld2411_get_out_pullup_enabled(void);
void ld2411_set_out_pullup_enabled(bool on);
bool ld2411_get_uart_presence_enabled(void);
void ld2411_set_uart_presence_enabled(bool on);

#ifdef __cplusplus
}
#endif
// Presence source selection
typedef enum {
    LD2411_SRC_OUT = 0,
    LD2411_SRC_UART = 1,
    LD2411_SRC_COMBINED = 2,
    LD2411_SRC_DISTANCE = 3,
} ld2411_presence_source_t;

int  ld2411_get_presence_source(void);
void ld2411_set_presence_source(int src);
int  ld2411_get_distance_thresh_cm(void);
void ld2411_set_distance_thresh_cm(int v);
bool ld2411_get_distance_presence_enable(void);
void ld2411_set_distance_presence_enable(bool on);

// Optional actions
void ld2411_action_force_publish(void);
void ld2411_action_reautobaud(void);
void ld2411_action_reset_tuning(void);
