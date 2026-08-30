# ESP32-C3 PlatformIO Labs

This project uses PlatformIO with the ESP-IDF framework. Each lab is a separate PlatformIO environment (`[env:labX_Y]` in `platformio.ini`) with its own source directory under `labs/`; a pre-build script (`select_lab.py`) automatically points the build at `labs/<env-name>/` based on which environment you run. Switch labs with `-e <env-name>` on any `pio` command — no other project files need to change.

## Labs

| Env | What it does | Hardware |
| --- | --- | --- |
| `lab1_2` | Hello World — prints chip model, core count, silicon revision, and flash size, then restarts. | None (onboard only) |
| `lab1_3` | Blinks the onboard WS2812 addressable RGB LED once a second via the `led_strip` RMT driver. | Onboard LED, GPIO8 |
| `lab2_2` | Reads temperature/humidity from an SHTC3 sensor over I2C using the new `i2c_master` API. | SHTC3, I2C SCL=8 SDA=10 |
| `lab3_2` | Drives a DFRobot RGB backlit LCD1602 over I2C, prints "Hello CSE121! / Nakamoto". | DFRobot RGB LCD1602, I2C SCL=0 SDA=1 |
| `lab3_3` | Combines the SHTC3 sensor and RGB LCD1602 into a live temperature monitor. | SHTC3 + LCD1602, I2C SCL=8 SDA=10 |
| `lab4_1` | Reads an ICM-42670-P accelerometer directly over I2C (legacy driver) and prints UP/DOWN/LEFT/RIGHT tilt direction with hysteresis smoothing. | ICM-42670-P, I2C SCL=8 SDA=10 |
| `lab4_2` | BLE HID peripheral — advertises as a Bluetooth mouse ("HIDD") and auto-sweeps the cursor right then left on a fixed ~5.5s timer once a host is paired/connected. No sensor input. | None (onboard only) |
| `lab4_3` | BLE HID peripheral — advertises as a Bluetooth mouse ("HID33") and moves the cursor based on accelerometer tilt (via the new `i2c_master` API), with a stationary-hold click gesture. | ICM-42670-P, I2C SCL=8 SDA=10 |
| `lab5_2` | Optical Morse code receiver — reads a photoresistor on ADC1 channel 3 (GPIO3), classifies light pulses into dots/dashes by timing, and decodes/prints letters. Morse unit = 20 ms. | Photoresistor, ADC1 ch3 / GPIO3 |
| `lab5_3` | Same receiver as `lab5_2`, tuned to a 18 ms Morse unit. | Same as `lab5_2` |
| `lab6_1` | Reads temperature (for speed-of-sound correction) from an SHTC3 sensor, then measures distance with an HC-SR04 ultrasonic sensor and prints `Distance: X.XX cm at Y.YYC`. | SHTC3 (I2C SCL=8 SDA=10) + HC-SR04 (TRIG=4, ECHO=5) |
| `lab7_1` | Weather station part 1 — connects to WiFi and periodically GETs the current temperature from `wttr.in`. | WiFi only |
| `lab7_2` | Weather station part 2 — reads the ESP32-C3's onboard die-temperature sensor and POSTs it as JSON to a Flask server (`labs/lab7_2/server.py`) on port 1234. | WiFi only |
| `lab7_3` | Weather station part 3 — GETs a configured location from the server (`GET /location`), queries `wttr.in` for that location's outdoor temperature, reads the onboard sensor, and POSTs all three back to the server. | WiFi only |

For labs 7.2/7.3, set `WIFI_SSID`, `WIFI_PASS`, and `SERVER_IP` in that environment's `build_flags` in `platformio.ini` before building. `SERVER_IP` must be reachable from the ESP32's WiFi network — see the `server.py` docstrings in each lab7 folder for setup notes (including a WSL/Windows port-forwarding gotcha if running the server inside WSL2).

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


