# **ESP32 Motion-Activated Lighting System**  

This project is an **ESP32-based motion-activated lighting system** that uses **UART-connected sensors** to detect movement and control lights. It also integrates **Wi-Fi connectivity** to send data to **Firebase** (though Firebase communication is not fully implemented).  

---

## **Features**
-  **Wi-Fi Connectivity**: Connects to a Wi-Fi network in **Station Mode (STA)**.  
-  **GPIO Control**: Controls LED indicators and light signals.  
-  **UART Communication**: Reads sensor data via UART.  
-  **Motion Detection**: Triggers **exterior and interior lights** when motion is detected.  
-  **Firebase Integration** *(incomplete)*: Logs data to Firebase.  
-  **Status LED Blinking**: Indicates activity.  

---

## **Core Functionalities**
###  **Wi-Fi Initialization (`wifi_init_sta`)**
- Connects the ESP32 to a Wi-Fi network using **SSID and password**.  
- Uses **Station mode (STA)** for connectivity.  
- Sets up the networking stack and initializes the Wi-Fi driver.  

###  **GPIO Configuration (`configure_gpio`)**
- Configures **GPIO pins** for LED outputs:  
  - `POWER_LED_PIN` (GPIO 25) - Always ON (indicating power).  
  - `LED_1` (GPIO 14) - Used as an indicator for Firebase communication.  
  - `LIGHT_SIGNAL_1` (GPIO 33) - Controls **exterior lights**.  
  - `LIGHT_SIGNAL_2` (GPIO 36) - Controls **interior lights**.  

###  **UART Communication Setup (`uart_init`)**
- Configures **two UART ports**:  
  - `UART_PORT_NUM_0` (GPIO 16 TX, GPIO 14 RX) → **Exterior sensor**.  
  - `UART_PORT_NUM_1` (GPIO 17 TX, GPIO 27 RX) → **Interior sensor**.  
- Enables data transmission between ESP32 and **two motion sensors**.  

###  **Sensor Handling Tasks (`exterior_sensor_task` and `interior_sensor_task`)**
- Reads sensor data over **UART** and triggers **light signals** when movement is detected.  
- **Exterior Sensor Task (`exterior_sensor_task`)**:
  - If data is received via **UART 0**:  
    - Turns ON **Light Signal 1** (GPIO 33).  
    - Keeps it ON for **10 minutes**.  
- **Interior Sensor Task (`interior_sensor_task`)**:
  - If data is received via **UART 1**:  
    - Turns ON **Light Signal 2** (GPIO 36).  
    - Keeps it ON for **10 minutes**.  
- After **10 minutes**, the respective light signal is turned OFF.  

###  **Firebase Communication (`send_to_firebase`)**
- Sends data to Firebase *(but Firebase communication is incomplete)*.  
- Toggles `LED_1` (blinks for **1 second**).  

###  **LED Blinking Task (`firebase_led_task`)**
- Continuously **blinks LED_1 every second**, possibly as a status indicator.  

###  **Main Application (`app_main`)**
- Configures **GPIO pins**.  
- Initializes **Wi-Fi** connection.  
- Sets up **UART communication**.  
- Starts the **Firebase LED blink task**.  
- Launches **sensor data processing tasks** (exterior and interior sensors).  

---

## **How It Works in Real-World Usage**
1. The **ESP32 connects to Wi-Fi**.  
2. It listens for **motion data** from two **PIR sensors** (or other motion sensors) via **UART**.  
3. When a **sensor detects motion**, the **corresponding light signal is turned ON** for **10 minutes**.  
4. The system **logs data** and can send updates to **Firebase** (though this part is incomplete).  
5. A **status LED blinks** to indicate activity.  

---

## **Potential Issues & Improvements**
###  **Security Risks**
- **The Firebase API key is hardcoded**, which is a security risk if exposed.  
- **Solution**: Move the API key to a **secure configuration file** or use **environment variables**.  

###  **Error Handling in Firebase Communication**
- `send_to_firebase()` **only logs the data** but does not send it to Firebase.  
- **Solution**: Use **`esp_http_client`** to properly send **HTTP POST requests** to Firebase.  

###  **UART Buffer Handling**
- The **UART read function does not handle buffer overflows** or **parsing errors**.  
- **Solution**: Implement **proper data parsing and validation** before processing sensor input.  

###  **Wi-Fi Stability Handling**
- The Wi-Fi connection **does not automatically reconnect** if it disconnects.  
- **Solution**: Add **Wi-Fi event handlers** for automatic reconnection.  

---

