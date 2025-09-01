# ESP32 IAQ + Network Portal

This project combines **Bosch BME680/BSEC air quality sensing** with a **Wi-Fi configuration portal** on the ESP32.  
It is designed to be **modular** and run different tasks on **separate ESP32 cores** for better performance and responsiveness.

---

## ✨ Features

- **BME680 sensor with BSEC library**
  - Indoor Air Quality (IAQ) index  
  - Estimated CO₂ equivalent  
  - Estimated breath VOC equivalent (bVOCeq)  
  - Temperature (°C)  
  - Relative humidity (%)  
  - Barometric pressure (hPa)  

- **Wi-Fi setup portal**
  - ESP32 starts as an Access Point (AP) when no Wi-Fi credentials are stored  
  - Web portal available at `http://192.168.4.1` to enter SSID and password  
  - Credentials are saved in NVS and automatically used on reboot  
  - AP SSID automatically ends with the **last 4 hex digits of the ESP32 MAC address**  
    (e.g. `ESP32-SETUP-3A7F`)  

- **Long-press reset**
  - Hold **GPIO0 (BOOT button)** for ≥3 seconds at boot to clear stored Wi-Fi credentials  
  - Device reboots into setup portal mode  

- **Dual-core task distribution**
  - **Core 1 (APP CPU):** Runs continuous BSEC sensor measurement task  
  - **Core 0 (PRO CPU):** Runs Wi-Fi portal, HTTP server, and button monitoring  

---

## 🛠️ Project Structure

esp32_iaq_portal/
├── components/
│ ├── app_cfg/ # Application config helper
│ ├── bme68x/ # Bosch BME680 driver
│ ├── bsec2/ # Bosch BSEC library + port
│ ├── sensor_iaq/ # IAQ measurement task
│ ├── utils/ # Utility functions
│ └── wifi_setup/ # Wi-Fi portal and button monitor
└── main/
└── main.c # Minimal launcher (delegates work to components)

yaml
Copy code

---

## 🚀 How it Works

1. On first boot, the ESP32 creates a Wi-Fi Access Point (AP).  
   - Connect to the AP using your phone/laptop.  
   - Open `http://192.168.4.1` to enter your Wi-Fi SSID and password.  

2. Device stores the credentials in NVS and reboots.  
   - If valid, the ESP32 connects to the configured Wi-Fi.  
   - If invalid, it reverts to AP mode again.  

3. The BME680 sensor is read continuously on **Core 1**, producing logs like:  
I (6400) SENSOR_IAQ: IAQ=45.0 (unreliable) CO2eq=600.2 ppm (unreliable)
bVOCeq=0.500 ppm (unreliable)
T=25.3°C RH=40.2% P=1013.5 hPa

yaml
Copy code

4. Long-press **GPIO0 (BOOT)** → clears Wi-Fi credentials → restarts in AP mode.

---

## 📋 Example Log Output

### First boot (no credentials)
I (123) MAIN: boot: init NVS
I (456) WIFI_SETUP: No Wi-Fi credentials found, starting AP mode
I (457) WIFI_SETUP: AP SSID: ESP32-SETUP-3A7F, password: 12345678
I (500) WIFI_SETUP: Web portal available at http://192.168.4.1

shell
Copy code

### After entering credentials in portal
I (105316) WIFI_SETUP: Rebooting to apply credentials...
I (1100) WIFI: Connected to SSID=myhomewifi
I (1101) WIFI: Got IP: 192.168.1.42

shell
Copy code

### Sensor data running on Core 1
I (3630) SENSOR_IAQ: IAQ=68.5 (reliable) CO2eq=720.0 ppm (reliable)
bVOCeq=0.520 ppm (reliable)
T=26.1°C RH=45.3% P=1012.9 hPa

graphql
Copy code

### Clearing Wi-Fi with GPIO0 (BOOT button)
W (2000) WIFI_SETUP: GPIO0 long-press detected → erasing Wi-Fi credentials
W (2001) WIFI_SETUP: Rebooting into AP setup mode

yaml
Copy code

---

## ⚡ Future Extensions

- Push sensor readings to AWS (API Gateway → Lambda → S3).  
- Add MQTT publishing for real-time monitoring.  
- Expose REST API endpoint on the ESP32 for local integrations.  

---

## 🔧 Build & Flash

```bash
idf.py set-target esp32
idf.py build
idf.py flash monitor
