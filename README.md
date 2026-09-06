# ESP32-C3 PlatformIO Labs

This project uses PlatformIO with the ESP-IDF framework. Each lab is a separate PlatformIO environment (`[env:labX_Y]` in `platformio.ini`) with its own source directory under `labs/`; a pre-build script (`select_lab.py`) automatically points the build at `labs/<env-name>/` based on which environment you run. Switch labs with `-e <env-name>` on any `pio` command — no other project files need to change.

## Labs

| Env | What it does | Hardware |
| --- | --- | --- |
| `lab1_2` | Hello World — prints chip model, core count, silicon revision, and flash size, then restarts. | None (onboard only) |
| `lab1_3` | Blinks the onboard WS2812 addressable RGB LED once a second via the `led_strip` RMT driver. | Onboard LED, GPIO8 |
| `lab2_2` | Reads temperature/humidity from an SHTC3 sensor over I2C using the new `i2c_master` API. | SHTC3, I2C SCL=8 SDA=10 |
| `lab3_2` | Drives a DFRobot RGB backlit LCD1602 over I2C, prints "Hello CSE121! / Nakamoto". | DFRobot RGB LCD1602, I2C SCL=0 SDA=1 |
| `lab3_3` | Combines the SHTC3 sensor and RGB LCD1602 into a live temperature monitor. | SHTC3 + LCD1602, I2C SDA=1 SCL=0 |
| `lab4_1` | Reads an ICM-42670-P accelerometer directly over I2C (legacy driver) and prints UP/DOWN/LEFT/RIGHT tilt direction with hysteresis smoothing. | ICM-42670-P, I2C SCL=8 SDA=10 |
| `lab4_2` | BLE HID peripheral — advertises as a Bluetooth mouse ("HIDD") and auto-sweeps the cursor right then left on a fixed ~5.5s timer once a host is paired/connected. No sensor input. | None (onboard only) |
| `lab4_3` | BLE HID peripheral — advertises as a Bluetooth mouse ("HID33") and moves the cursor based on accelerometer tilt (via the new `i2c_master` API), with a stationary-hold click gesture. | ICM-42670-P, I2C SCL=8 SDA=10 |
| `lab5_2` | Optical Morse code receiver + onboard LED transmitter — self-contained, no Raspberry Pi or PC screen needed. Photoresistor on ADC1 channel 3 (GPIO3) reads whatever the onboard LED (GPIO4, blinks `TX_MESSAGE`) sends and decodes/prints letters. Morse unit = 20 ms (original spec). | Photoresistor (ADC1 ch3 / GPIO3) + LED (GPIO4) |
| `lab5_3` | Same as `lab5_2` — self-contained LED transmitter + photoresistor receiver, tuned to an 18 ms Morse unit (original spec). | Same as `lab5_2` |
| `lab5_loopback` | Self-contained Morse test — no Raspberry Pi needed. Runs the `lab5_2` photoresistor receiver *and* an LED transmitter (blinks `TX_MESSAGE`, default "SOS") on the same board; point the LED at the photoresistor and shield both from ambient light. An LCD1602 mirrors the serial monitor — rolling decoded text across both lines — so you can watch it work without a serial monitor open. Morse unit = 55 ms; with the LCD sharing the breadboard, 52ms and below decode as garbage but 55ms+ is clean. | Photoresistor (ADC1 ch3 / GPIO3) + LED (GPIO4) + DFRobot RGB LCD1602 (I2C SDA=1 SCL=0) |
| `lab6_1` | Reads temperature (for speed-of-sound correction) from an SHTC3 sensor, then measures distance with an HC-SR04 ultrasonic sensor and prints `Distance: X.XX cm at Y.YYC`. | SHTC3 (I2C SCL=8 SDA=10) + HC-SR04 (TRIG=4, ECHO=5) |
| `distance_lcd` | lab6 + lab3 combined — same temperature-corrected ultrasonic distance measurement, displayed live on the LCD1602 instead of just printed. LCD gets the one hardware I2C bus; SHTC3 uses its own bit-banged software I2C bus (same trick as `lab3_3`), leaving TRIG/ECHO free on plain GPIOs. | DFRobot RGB LCD1602 (I2C SDA=1 SCL=0) + SHTC3 (software I2C SDA=10 SCL=8) + HC-SR04/RCWL-1601 (TRIG=4, ECHO=5) |
| `lab7_1` | Weather station part 1 — connects to WiFi and periodically GETs the current temperature from `wttr.in`. | WiFi only |
| `lab7_2` | Weather station part 2 — reads the ESP32-C3's onboard die-temperature sensor and POSTs it as JSON to a Flask server (`labs/lab7_2/server.py`) on port 1234. | WiFi only |
| `lab7_3` | Weather station part 3 — GETs a configured location from the server (`GET /location`), queries `wttr.in` for that location's outdoor temperature, reads the onboard sensor, and POSTs all three back to the server. | WiFi only |
| `weather_lcd` | Final weather station — combines lab3 (DFRobot RGB LCD1602) and lab7 (WiFi + wttr.in HTTP GET). One `wttr.in` request returns location, temperature, condition, and humidity; the LCD cycles every 3s between a location screen and a weather screen, re-fetching from `wttr.in` every 60s. Cycles through 4 fixed cities (Chicago, New York, Japan, Paris): the **BOOT** button (GPIO9) is polled live and scrolls down to the previous city instantly; the **RESET/EN** button reboots the chip (it's wired to the hardware reset line, not a GPIO, so it can't be polled) and the city index — persisted in NVS flash — advances to the next city on every boot, acting as "scroll up". | DFRobot RGB LCD1602, I2C SDA=1 SCL=0; onboard BOOT/RESET buttons |

For labs 7.2/7.3, set `WIFI_SSID`, `WIFI_PASS`, and `SERVER_IP` in that environment's `build_flags` in `platformio.ini` before building. `SERVER_IP` must be reachable from the ESP32's WiFi network — see the `server.py` docstrings in each lab7 folder for setup notes (including a WSL/Windows port-forwarding gotcha if running the server inside WSL2).

For `weather_lcd`, set `WIFI_SSID` and `WIFI_PASS` in its `build_flags` in `platformio.ini`. The city list (Chicago, New York, Japan, Paris) is hardcoded in the `CITIES[]` array in `labs/weather_lcd/main.cpp`.

`lab5_2`/`lab5_3`/`lab5_loopback` were all made self-contained with an onboard LED transmitter (GPIO4) so none of them need an external Raspberry Pi or PC screen. Two real bugs had to be fixed: (1) `CONFIG_FREERTOS_HZ` is set to 1000 in each env's `sdkconfig.<env>` — the project default of 100 rounds sub-10ms `vTaskDelay` calls down to 0, breaking fine-grained timing; (2) `THRESHOLD_MARGIN` was lowered from 50 to 20 — the old margin was calibrated against a dark baseline that turned out to be miscalibrated for the actual light/dark contrast, so `light_threshold` sat above the LED's actual lit reading and almost never triggered. With both fixed, `lab5_2`/`lab5_3` (no LCD on the breadboard) run cleanly at their original spec'd 20ms/18ms units — no compensation needed, the sensor's edges are sharp. `lab5_loopback` has the LCD sharing the same breadboard, which costs real margin (52ms and below decode as garbage, 55ms+ is clean) and still needs a small first-pulse compensation (the first pulse after a letter-or-longer gap measures a bit short) to decode perfectly at 55ms. Wiring: photoresistor divider on GPIO3 (3.3V → photoresistor → node → resistor → GND, node to GPIO3), LED on GPIO4 (anode → GPIO4, cathode → resistor → GND), LED positioned against the photoresistor with both shielded from ambient light.

## Prerequisites

- ESP32-C3 board
- USB data cable
- Windows with WSL2 and Ubuntu
- PlatformIO installed in WSL at `~/.platformio/penv/bin/pio`
- `usbipd-win` installed in Windows


cd ~/PlatformIO/Projects/ESP32-C3-mini-1


## 2. Connect the ESP32 to WSL

Run the following commands in **Windows PowerShell**. Do not run them in Ubuntu/WSL.

List USB devices:

powershell
usbipd list

Find the row for the ESP32. It may be displayed as `USB Serial Device`, `USB JTAG/serial debug unit`, or similar. Copy its `BUSID`, such as `2-12`.

Share and attach the device to WSL:

powershell
usbipd list
usbipd bind --busid 2-12
usbipd attach --wsl --busid 2-12

Replace `2-12` with the actual BUSID. The `bind` command may require Administrator PowerShell.

Its state should be `Attached`.

## 3. Find the serial port in WSL

Return to Ubuntu/WSL and run:

ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
export ESP_PORT=/dev/ttyACM0

Replace `/dev/ttyACM0` with the port shown on your system.

## 4. Choose and build a lab

Run in Ubuntu/WSL from the project directory:

cd ~/PlatformIO/Projects/ESP32-C3-mini-1
export LAB=lab3_3
~/.platformio/penv/bin/pio run -e "$LAB"

## 5. Upload the firmware

~/.platformio/penv/bin/pio run -e "$LAB" -t upload --upload-port "$ESP_PORT"

## 6. Monitor serial output

After upload completes, run:

~/.platformio/penv/bin/pio device monitor --port "$ESP_PORT" --baud 115200

## 7. Detach the device when finished

Run this in Windows PowerShell:

powershell
usbipd detach --wsl --busid 2-12


