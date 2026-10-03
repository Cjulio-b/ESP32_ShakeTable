# ESP32 Shake Table 🫨

Welcome to the **ESP32 Shake Table** project repository. This project consists of the development and control of a Shake Table using an **ESP32** microcontroller (Adafruit Huzzah32) and a graphical user interface through a **Nextion HMI** display.

**System Architecture Note:** This microcontroller acts as the primary motion controller (**MCU1**). It is designed to work in tandem with a separate Data Acquisition (DAQ) unit (**MCU2**), which is developed in a parallel project: `ESP32_DAQ_Controller`.

The shake table is actuated by stepper motors and is designed for structural testing, earthquake simulations, and academic research.

---

## 🛠 Hardware Setup
- **Microcontroller:** Adafruit Huzzah32 (ESP32)
- **Graphical Interface (HMI):** Nextion Display
- **Actuators:** Stepper Motors (e.g., NEMA 17/23/34)
- **Motor Drivers:** *(Currently evaluating DRV8825, TMC2208/2209/2225 or TB6600/TB67S109)*
- **Expanders:** MCP23017 (I2C GPIO Expander)

## 🚀 Getting Started

### Prerequisites
- Install [ESP-IDF (v5.5.1 or higher)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)
- Code Editor (e.g., VS Code)
- Git

### Build and Flash
To configure and compile the project, use the following commands in the ESP-IDF terminal (`idf.py`):

1. **Menu Configuration (Optional):**
   ```bash
   idf.py menuconfig
   ```
2. **Build the Code:**
   ```bash
   idf.py build
   ```
3. **Flash to ESP32:**
   ```bash
   idf.py flash
   ```
4. **Monitor Logs (Serial Monitor):**
   ```bash
   idf.py monitor
   ```
*(Tip: You can use `idf.py build flash monitor` in a single command)*

---

## 📂 Project Structure & Code Explanation

The core logic of the project is located in the `main` directory. Below is a breakdown of the key files, what they do, and their importance to the project:

### 1. Core System & Entry Point
- **`ESP32_ShakeTable.c`**: This is the main entry point of the application (`app_main`). **Importance:** It orchestrates the initialization of all other modules (Wi-Fi, Motors, HMI, I2C) and runs the main loop of the system.
- **`functions.h`**: Central header file containing shared macros, structures, and function declarations. **Importance:** Keeps the project organized by centralizing shared definitions.

### 2. Motor Control & Movement
- **`kinematics.c` / `.h`**: Handles the mathematical calculations for movement, speeds, and acceleration profiles to simulate earthquakes or specific frequencies. **Importance:** Crucial for translating theoretical movement data into physical motor steps.
- **`stepper_motor_rmt.c`**: Implements stepper motor control using the ESP32's internal RMT (Remote Control) peripheral. **Importance:** The RMT peripheral allows for highly precise and hardware-accelerated pulse generation, which is essential for smooth motor movement at high speeds without blocking the CPU.
- **`l298n_stepper.c` / `.h`**: Driver code for controlling motors via the L298N module. *(Note: This module is not actively used in the current project architecture; it was only part of the initial testing phases).*
- **`stepper_basic_test.c`**: A simple test script used to verify motor pinouts and basic functionality before running complex kinematics.

### 3. User Interface (HMI) & Communication
- **`nextion.c`**: Manages all UART communication with the Nextion HMI display. It processes touch events from the screen and updates UI elements. **Importance:** This is the bridge between the user and the machine, allowing for real-time control and monitoring.
- **`uart_async_rxtxtasks_main.c`**: Handles asynchronous UART communication tasks to ensure that receiving data from the Nextion display does not block the main motor control loop.

### 4. Networking & Web Interface
- **`wifi.c`**: Configures the ESP32 to connect to a Wi-Fi network or act as an Access Point (AP).
- **`http_server.c`**: Sets up an embedded web server on the ESP32. **Importance:** Allows remote control, monitoring, and potentially uploading new earthquake simulation profiles directly via a web browser.

### 5. Hardware Interfacing & Configuration
- **`GPIO_config.c`**: Configures the standard General Purpose Input/Output pins of the ESP32.
- **`i2c_bus.c` / `.h`**: Manages the I2C bus communication.
- **`mcp23017.c` / `.h`**: Driver for the MCP23017 GPIO expander. **Importance:** Used to add more inputs/outputs via I2C since the ESP32 has limited pins available.
- **`config_manager.c` & `esp_littlefs.c`**: Manages non-volatile storage using the LittleFS file system. **Importance:** Allows the ESP32 to save user configurations, calibration data, and Wi-Fi credentials persistently across reboots.

---

## 📚 Developer Quick Reference (Cheatsheet)

### ESP-IDF Commands
```bash
idf.py create-project -p . <project name>  # Create new project
code .                                     # Open folder in VS Code
start idf.py menuconfig                    # Start menuconfig in a new window
idf.py fullclean                           # Clean the build directory
idf.py set-target <target>                 # Set target MCU
```
**Useful ESP-IDF Links:**
- [idf.py Commands](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/tools/idf-py.html)
- [Logging System](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/log.html)
- [ESP32 Official Documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)
- [Adafruit Huzzah32 Datasheet](https://cdn-learn.adafruit.com/downloads/pdf/adafruit-huzzah32-esp32-feather.pdf)

### Git Commands
```bash
git status
git add .
git commit -m "write text"
git log      # (Press "q" to quit)
git push
```

### Nextion HMI - Documentation
- [General Instruction Set](https://nextion.tech/instruction-set/)
- [Instruction Set (S3)](https://nextion.tech/instruction-set/#s3)
- [Editor Guide](https://nextion.tech/editor_guide/)

### Using menuconfig in PowerShell
If you are using PowerShell and encounter execution policy issues, run:
```powershell
cd C:\Users\cjulio\esp\v5.5.1\esp-idf
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\export.ps1
cd C:\Users\cjulio\Documents\cjulio\ESP32_ShakeTable
idf.py menuconfig
```

### Image Processing (FFmpeg)
To convert image/video resolution for the Nextion display:
1. Install `ffmpeg` via CMD.
2. Navigate to your image directory.
3. Example command:
```cmd
cd C:\Users\cjulio\Documents\cjulio\ESP32_ShakeTable\HMI_NEXTION\imagens\utility
ffmpeg -i 1_Header.png -vf "scale=95:116" -c:a copy 1_Header_95_196.png
```
