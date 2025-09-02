# Presence Sensor (ESP32‑C3 + HLK‑LD2420)

ESPHome has been fussy with LD2420 presence sensors lately (random regressions, timing quirks, discovery breakage) and I got tired of chasing it. This repo is a tiny, focused ESP‑IDF firmware that does one thing well: read the LD2420 reliably and publish clean MQTT with Home Assistant discovery.

No YAML, no surprises—just fast boots, stable presence, optional distance, and diagnostics (RSSI/uptime). Configure `config/secrets.h`, build/flash with ESP‑IDF, and it shows up in HA.

Notes:
- MQTT publishes now wait until the client is connected (less noisy logs on boot/reconnect).
- Home Assistant discovery prefix is configurable via `HA_DISCOVERY_PREFIX` in `config/secrets.h` (defaults to `homeassistant`).

**Sensor Models**
- LD2420: Full support with UART autobaud + heuristics and OUT/OT2 debounce/hold.
- HLK‑LD2411: Initial support via digital OUT presence (UART parsing TBD).
  - Enable by setting `#define USE_LD2411 1` in `config/secrets.h`.
  - Wiring and pins follow the same per‑target defaults listed below.

HLK‑LD2411 (ESP32‑C6) Pin Map
- VIN → 5V
- GND → GND
- TXD → GPIO17 (ESP RX)
- RXD → GPIO16 (ESP TX)
- OUT → GPIO14 (presence)

Notes
- Firmware uses `UART1` for the sensor; keep the console on USB‑CDC.
- If presence appears inverted, toggle `LD2420_OT2_ACTIVE_HIGH` in `config/secrets.h`.

**Target Toggle (C3/C6)**
- Default target: `esp32c3`.
- Override at build time:
  - With CMake: `idf.py -DCHIP=esp32c6 build` or `idf.py -DIDF_TARGET=esp32c6 build`
  - With env var: `IDF_TARGET=esp32c6 idf.py build`
- You can still use the standard: `idf.py set-target esp32c6`.
- The `sdkconfig.defaults` no longer pins the target; IDF picks it from the above.

**Pin Selection Tips (C3/C6)**
- UART is matrix‑routable: you can place TX/RX on most GPIOs.
- Prefer non‑special pins; avoid strapping (boot), JTAG, USB, and flash pins.
- Defaults in `config/secrets.h.template` select safe per‑target pins:
  - ESP32‑C6: TX=IO16, RX=IO17, OT2=IO14.
  - ESP32‑C3: TX=IO21, RX=IO20, OT2=IO4.
- If your C6 board silk labels “UART TX/RX” on 16/17, use those for the sensor.
- Keep the console on USB‑Serial/JTAG and use `UART1` for the sensor to avoid logs on the same bus.

Pins to avoid on ESP32‑C6 (when possible): IO4‑7 (JTAG), IO8‑9 (strapping), IO12‑13 (USB), IO18‑19 (flash).
