# energy-management-system-with-wokwi-and-blynk
# System Functionalities

## 1. Heating and Cooling (Gas Heating & AC)
**Function:** Automatically regulate indoor temperature based on motion detection and pre-defined temperature thresholds.  
**Conditions:**  
- **If motion is detected:**  
  - Temperature < 20°C: Activate gas heating (Heater LED ON).  
  - Temperature > 25°C: Activate AC (AC LED ON).  
  - Temperature between 20°C and 25°C: Keep both systems OFF.  
- **If no motion is detected for >30 minutes:** Deactivate heating/AC (LEDs OFF) to save energy.  

## 2. Lighting Optimization  
**Function:** Dynamically adjust LED lighting based on ambient light levels to reduce electricity consumption.  
**Conditions:**  
- If natural light >70%: Turn off LEDs (LED OFF).  
- If natural light <30%: Set LEDs to 100% brightness (LED ON).  

## 3. Water Consumption Management  
### 3.1 Leak Detection  
**Function:** Detect and respond to water leaks in real-time to prevent water wastage.  
**Conditions:**  
- If water flow is detected continuously for >30 minutes without faucet usage:  
  - Shut off the main water valve (Servo ON).  
  - Alert the user (Buzzer ON).  

### 3.2 Smart Irrigation  
**Function:** Automate outdoor watering based on soil moisture levels and weather forecasts.  
**Conditions:**  
- If soil moisture <30% and no rain is forecasted: Activate irrigation (LED ON).  
- If soil moisture >70% or rain is expected: Keep irrigation OFF (LED OFF).  

## 4. Gas Consumption Monitoring  
**Function:** Monitor gas usage and alert the user if it exceeds a predefined threshold.  
**Condition:**  
- If gas level exceeds the threshold: Activate buzzer (Buzzer ON).  
