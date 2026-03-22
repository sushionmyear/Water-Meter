# Water Meter Firmware

ESP32-based water meter firmware for reading a water meter with a LIS3MDL magnetic sensor and publishing usage to MQTT and Home Assistant.

It includes:

- a browser-based Web UI for setup and tuning
- Home Assistant MQTT discovery
- browser-based OTA firmware updates
- persistent settings stored on the device
- flow-rate and leak-state reporting

![Web UI preview](docs/webui-preview.svg)

## What It Does

The firmware watches the magnetic field near the meter, detects pulses as the dial moves, converts those pulses into gallons, and publishes the results over MQTT.

Main outputs:

- total gallons
- flow rate in GPM
- leak state
- RSSI
- optional magnetic debug value

## How It Works

```mermaid
flowchart LR
  A["Water meter dial movement"] --> B["LIS3MDL magnetic sensor"]
  B --> C["ESP32 pulse detection"]
  C --> D["Gallons + flow calculation"]
  D --> E["MQTT topics"]
  D --> F["Web UI status page"]
  E --> G["Home Assistant discovery + entities"]
  F --> H["Settings, reboot, counter reset, OTA"]
```

## Web UI

The built-in Web UI is designed to handle normal day-to-day management without code changes.

From the Web UI you can:

- change Wi-Fi settings
- change MQTT broker settings
- tune pulse detection thresholds
- adjust gallons-per-pulse calibration
- change publish and persistence intervals
- reset the pulse counter
- reboot the device
- upload new firmware over the air

## Hardware

Required hardware:

- ESP32 Dev Module / ESP32-WROOM-32U
- Adafruit LIS3MDL
- stable 5V USB power supply

ESP32 SPI wiring used by this project:

| LIS3MDL | ESP32 |
| --- | --- |
| `SCK` | `GPIO18` |
| `MISO` / `SDO` | `GPIO19` |
| `MOSI` / `SDA` | `GPIO23` |
| `CS` | `GPIO5` |
| `3.3V` | `3.3V` |
| `GND` | `GND` |

## Quick Start

### 1. Set first-boot defaults

For a brand-new board, edit the placeholder defaults in `src/WaterMeterApp.cpp` before flashing:

- `DEFAULT_WIFI_SSID`
- `DEFAULT_WIFI_PASS`
- `DEFAULT_MQTT_HOST`
- `DEFAULT_MQTT_USER`
- `DEFAULT_MQTT_PASS`

These are only starter values. After the device is on your network, you can change everything from the Web UI.

### 2. Build

```powershell
pio run -e esp32dev
```

If `pio` is not on your PATH, build from PlatformIO in VS Code or run the PlatformIO executable directly from your local installation.

### 3. Flash over USB the first time

Build output:

- firmware image: `.pio/build/esp32dev/firmware.bin`

Typical first flash:

```powershell
pio run -e esp32dev -t upload
```

### 4. Open the device on your network

After boot, open the serial monitor at `115200` baud and look for the IP address.

Then visit:

```text
http://<device-ip>/
```

### 5. Finish setup in the Web UI

Recommended first changes:

- confirm Wi-Fi settings
- confirm MQTT broker address
- choose your final `Base Topic`
- choose your final `Device ID`
- choose your final `Device Name`
- set `Mag Debug Publish (ms)` to `0` for normal use

## OTA Firmware Updates

After the first USB flash, future updates can be done from the browser.

1. Build the firmware.
2. Open the device IP in your browser.
3. In `Firmware OTA`, upload `.pio/build/esp32dev/firmware.bin`.
4. Wait for the device to reboot.

OTA updates do not erase saved settings.

## Home Assistant

The firmware publishes Home Assistant MQTT discovery data automatically after connecting to MQTT.

Typical entities created:

- total water usage
- current flow rate
- Wi-Fi RSSI
- leak state
- magnetic debug sensor

Important settings:

- `Home Assistant Prefix`: usually `homeassistant`
- `Base Topic`: the live topic for this device
- `Device ID`: unique ID used in entity naming
- `Device Name`: friendly name shown in Home Assistant

## Important Settings

These are the settings most users are likely to adjust:

- `Gallons Per Pulse`: calibration value for your meter
- `Mag High Threshold`: level that counts a pulse
- `Mag Low Threshold`: level that re-arms the next pulse
- `Pulse Lockout (ms)`: minimum spacing between pulses; lower values allow higher max flow
- `Sensor Poll (ms)`: how often the LIS3MDL is sampled
- `Flash Save Interval (ms)`: how often pulse count is saved to flash
- `Leak Minimum Flow (gpm)`: minimum sustained flow before leak timing starts
- `Leak Minimum Duration (ms)`: how long flow must persist before leak state turns on

## MQTT Topics

With `Base Topic = home/water_meter`, the main topics are:

- `home/water_meter/status`
- `home/water_meter/total_gallons`
- `home/water_meter/flow_gpm`
- `home/water_meter/rssi`
- `home/water_meter/leak`
- `home/water_meter/mag_debug`
- `home/water_meter/cmd`

If `Enable MQTT RESET command` is turned on, publishing `RESET` to `<base_topic>/cmd` clears the counter.

## Troubleshooting

### Web UI does not load

- Check the serial monitor for the IP address.
- Confirm the ESP32 joined your Wi-Fi.
- Verify you are opening `http://<device-ip>/` on the same network.

### Sensor shows as missing

- Recheck the SPI wiring.
- Confirm the LIS3MDL is powered from `3.3V`, not `5V`.
- Confirm `CS` is connected to `GPIO5`.

### Flow is too low or pulses are missed

- Lower `Pulse Lockout (ms)` carefully.
- Lower `Sensor Poll (ms)` if needed.
- Recheck sensor placement near the meter magnet.

### False counts or noisy readings

- Raise `Mag High Threshold`.
- Lower `Mag Low Threshold` only if the sensor is failing to re-arm.
- Set `Mag Debug Publish (ms)` to a small value temporarily so you can inspect the live magnetic value over MQTT.

### OTA upload fails

- Make sure you upload the matching ESP32 firmware binary.
- Wait for the current page to finish loading before uploading.
- Retry after a manual reboot if the device was left in an odd state by a previous failed update.

## Project Layout

- `platformio.ini` - PlatformIO environment for the ESP32 build
- `src/main.cpp` - Arduino entrypoint
- `src/WaterMeterApp.cpp` - firmware logic, Web UI, MQTT, OTA, persistence
- `include/WaterMeterApp.h` - app entrypoint declarations

## Notes

- Default network and MQTT values in the source are placeholders for first-time setup.
- Saved settings persist across reboot and OTA updates.
- `.pio/` and `.vscode/` are ignored by Git.
