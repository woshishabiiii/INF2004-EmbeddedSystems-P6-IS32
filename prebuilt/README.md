# Prebuilt images

Built from this tree on 2026-10-14 with the default values in the `*_cfg.h`
headers (nothing measured yet), dual-core, USB serial console, no MQTT.

| File | Same as | Use |
|---|---|---|
| `car_usb_console.uf2` | `cd build_make; make` | the car; telemetry printed on the USB console |
| `car_usb_console_selftest.uf2` | `make TEST=1` | the same, plus every subsystem's `[TEST]` checks at power-up |

Flash: hold BOOTSEL while plugging in the Pico W, copy the file to the
RPI-RP2 drive. Open the USB serial port at 115200 (CAR_GUIDE.md section 3).

There is no prebuilt MQTT image: it needs your WiFi name, password and
broker address in `config/wifi_secrets.h`, then `make NET=lwip MQTT=1`.
After you change any `MEASURE` value, rebuild instead of using these.
