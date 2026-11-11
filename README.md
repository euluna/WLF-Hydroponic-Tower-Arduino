# WLF Gardens - Hydroponic Tower Sensor Master

This repository contains the C++ firmware for the WLF Gardens hydroponic tower. The code runs on the main microcontroller to interface with environmental sensors, flow meters, and relays. 

This project is built and managed using **PlatformIO** instead of the standard Arduino IDE.

## What is PlatformIO (PIO)?

PlatformIO is a professional, open-source build tool for embedded development. We use it because it automatically manages library dependencies (like DallasTemperature, DHT, and Adafruit Unified Sensor), handles board configurations via `platformio.ini`, and integrates seamlessly with VS Code.

### How to Use PlatformIO

1. **Install VS Code:** Download and install [Visual Studio Code](https://code.visualstudio.com/).
2. **Install the PIO Extension:** Open VS Code, go to the Extensions tab (`Ctrl+Shift+X`), search for **PlatformIO IDE**, and install it. Restart VS Code if prompted.
3. **Open the Project:** Click `File > Open Folder` and select the `WLF_Sensor_Master` directory (the folder containing `platformio.ini`).
4. **Build and Upload:**
   * Look for the PlatformIO alien head icon in the left sidebar or the checkmark/arrow icons in the bottom blue status bar.
   * Click the **Checkmark (✓)** to build/compile the firmware.
   * Connect your microcontroller via USB and click the **Right Arrow (→)** to upload the firmware.
5. **Serial Monitor:** Click the **Plug icon** in the bottom status bar to open the serial monitor and view live sensor output.

---

## Using the Python Script

The included Python script is used to interface with the microcontroller over serial. 

### 1. Create and Activate a Virtual Environment
Using a virtual environment keeps the script isolated from your main system. Open your terminal in the project folder and run the following commands to create and activate it:

```bash
# 1. Create the virtual environment
python -m venv venv

# 2. Activate the virtual environment based on your operating system:

# Windows (Command Prompt):
venv\Scripts\activate.bat

# Windows (PowerShell):
venv\Scripts\Activate.ps1

# macOS / Linux:
source venv/bin/activate
```
### 2. Run the Script
Make sure your microcontroller is plugged in and that the PlatformIO serial monitor is closed. If the PIO monitor is open, the Python script will be blocked from accessing the COM port.
```bash
python your_script_name.py
