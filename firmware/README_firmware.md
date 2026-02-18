## ⚡ Embedded Firmware Modifications (OpenCR)
To support **Sharp GP2Y0D810Z0F Digital Distance Sensors** for cliff detection, the default OpenCR firmware (C++) was modified. The standard `ollo` library was replaced with a custom driver implementation to handle digital GPIO signals with lower latency.

* **File:** `firmware/opencr_modified/turtlebot3_sensor.cpp`
* **Technical Implementation:**
    * **Direct GPIO Access:** Replaced the proprietary `ollo_.read()` analog wrapper with direct `digitalRead()` on **GPIO 15 and 16**.
    * **Hardware-Level Sensor Fusion:** Implemented a logic gate `(Sensor1_GND && Sensor2_GND)` directly on the microcontroller.
    * **Safety Logic:** The function returns `0` (Safe) only if **both** sensors detect the ground. If either sensor detects a cliff (Logic 1), the system triggers a stop condition immediately, providing redundant safety. 
