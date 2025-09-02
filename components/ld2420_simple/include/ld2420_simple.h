#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ld2420_simple_cb_t)(bool present, int distance_mm);

typedef struct {
    uart_port_t uart_num;       // UART port (e.g., UART_NUM_1)
    gpio_num_t  uart_tx;        // ESP TX -> LD2420 RX
    gpio_num_t  uart_rx;        // ESP RX <- LD2420 TX
    int         baud;           // fixed baud, e.g., 115200
    int         sample_ms;      // loop period, e.g., 50
    int         hold_ms;        // keep present true after last detection, e.g., 500..2000

    // Optional: use OT2 digital presence pin
    gpio_num_t  ot2_gpio;       // GPIO_NUM_NC to disable
    bool        ot2_active_high;// true if high = occupied
    int         debounce_ms;    // edge debounce for OT2

    // Optional boot/probe transmissions (sent once on start)
    const char *boot_tx_ascii;  // e.g., "UARTRPT=1\r\n"
    const char *boot_tx_hex;    // e.g., "FD FC FB FA 08 00 12 00 00 00 64 00 00 00 04 03 02 01"
    int         boot_tx_repeat; // default 2
    int         boot_tx_delay_ms; // default 20

    ld2420_simple_cb_t cb;      // presence publisher (present, distance_mm)
} ld2420_simple_cfg_t;

esp_err_t ld2420_simple_init(const ld2420_simple_cfg_t *cfg);
esp_err_t ld2420_simple_start(void);
void      ld2420_simple_stop(void);

bool ld2420_simple_get_present(void);
int  ld2420_simple_get_last_distance_mm(void);

#ifdef __cplusplus
}
#endif
