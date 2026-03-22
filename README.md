# Water Meter Firmware

ESP32-based water meter firmware with:

- LIS3MDL pulse sensing over SPI
- MQTT publishing and Home Assistant discovery
- Web UI for settings
- browser-based OTA firmware updates

## Project Layout

- `platformio.ini` - PlatformIO environment for the ESP32 build
- `src/main.cpp` - Arduino entrypoint
- `src/WaterMeterApp.cpp` - firmware logic, Web UI, MQTT, OTA, persistence
- `include/WaterMeterApp.h` - app entrypoint declarations

## Hardware

- ESP32 Dev Module / ESP32-WROOM-32U
- Adafruit LIS3MDL

ESP32 SPI wiring used by this project:

- `SCK` -> `GPIO18`
- `MISO` -> `GPIO19`
- `MOSI` -> `GPIO23`
- `CS` -> `GPIO5`
- `3.3V` -> `3.3V`
- `GND` -> `GND`

## First-Time Setup

Settings are stored on the device after first boot.

For a brand-new board, update the placeholder defaults in `src/WaterMeterApp.cpp` before flashing, especially:

- `DEFAULT_WIFI_SSID`
- `DEFAULT_WIFI_PASS`
- `DEFAULT_MQTT_HOST`
- `DEFAULT_MQTT_USER`
- `DEFAULT_MQTT_PASS`

After the board is on your network, use the Web UI to change settings without editing code.

## Build

```powershell
pio run -e esp32dev
```

The compiled OTA image will be here:

- `.pio/build/esp32dev/firmware.bin`

If `pio` is not on your PATH, build from PlatformIO in VS Code or run the PlatformIO executable directly from your local installation.

## OTA Update

1. Open the device IP in your browser.
2. Use the `Firmware OTA` section.
3. Upload `.pio/build/esp32dev/firmware.bin`.
4. Wait for the device to reboot.

## Publish To GitHub Safely

This local repo contains old personal Wi-Fi and MQTT values in its Git history, so do not push the existing `.git` history to a public GitHub repo.

Safest method:

1. Create a new empty folder somewhere else.
2. Copy this project into it, but do not copy the `.git` folder.
3. In the new folder, run:

```powershell
git init
git add .
git commit -m "Initial public release"
git branch -M main
git remote add origin https://github.com/<your-user>/<your-repo>.git
git push -u origin main
```

## Notes

- `.pio` is ignored by Git and should not be committed.
- If the old Wi-Fi or MQTT credentials are still in use, rotate them before making a public repo.
