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

    // App callback
    ld2420_presence_cb_t cb;
} ld2420_cfg_t;

esp_err_t ld2420_init(const ld2420_cfg_t *cfg);
esp_err_t ld2420_start(void);
void      ld2420_stop(void);

// Helpers
bool ld2420_get_present(void);
int  ld2420_get_last_distance_mm(void);
bool ld2420_uart_active(void);       // true if we detected a live UART stream
int  ld2420_active_baud(void);       // returns 0 if unknown

#ifdef __cplusplus
}
#endif
