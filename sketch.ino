#define BLYNK_TEMPLATE_ID "TMPL2S0nBM5e3"
#define BLYNK_TEMPLATE_NAME "ems"
#define BLYNK_AUTH_TOKEN "Tz-YHr_ev527HImGl9MD4eSQ"
#include <WiFi.h>
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "DHTesp.h"
#include <ESP32Servo.h>
#include "time.h"
#include <BlynkSimpleEsp32.h>  // Include Blynk library for ESP32
// Virtual pins for Blynk control
#define V_HEATER_PIN V1   // Virtual pin for Heater LED control
#define V_AC_PIN V2       // Virtual pin for AC LED control
#define V_LIGHT_PIN V3    // Virtual pin for Light LED control
#define V_IRRIGATION_PIN V4  // Virtual pin for Irrigation LED control
#define V_SERVO_PIN V5 
#define V_TEMPERATURE V6        // Temperature
#define V_HUMIDITY V7           // Humidity
#define V_LDR V8               // LDR (Light Sensor)
#define V_WATER_CONSUMPTION V9  // Water Consumption
#define V_GAS_LEVEL V10          // Gas Level
#define V_PIR V11
// Pin definitions
#define DHT1_PIN 21  // Temperature sensor pin
#define DHT2_PIN 19  // Soil moisture sensor pin (DHT2)
#define PIR_PIN 26   // PIR motion sensor pin
#define LDR_PIN 32    // Light sensor pin
#define POTENTIOMETER_1_PIN 34  // Gas level sensor pin
#define POTENTIOMETER_2_PIN 35  // Water consumption sensor pin
#define BUZZER_GAS_PIN 15  // Buzzer for gas consumption
#define BUZZER_WATER_PIN 2 // Buzzer for water leak
#define HEATER_LED_PIN 0   // LED for heater
#define AC_LED_PIN 4        // LED for AC
#define LIGHT_LED_PIN 16    // LED for light control
#define IRRIGATION_LED_PIN 5 // LED for irrigation
#define SERVO_PIN 18

#define AWS_IOT_PUBLISH_TOPIC "iot/data"
#define AWS_IOT_SUBSCRIBE_TOPIC "iot/commands"

// Initialize DHT sensors
DHTesp dhtSensor1;
DHTesp dhtSensor2;

// Variables for sensor readings
int gasLevel = 0;
int waterConsumptionLevel = 0;
int ldrValue = 0;
int pirValue = 0;
int soilMoisture = 0; // Soil moisture from DHT2 (humidity)

// Variables for Wi-Fi and MQTT connection
WiFiClientSecure net;
PubSubClient client(net);

// Timer for motion detection (no motion timeout)
unsigned long lastMotionTime = 0;
const unsigned long motionTimeout = 2000; // 3S in milliseconds

// Gas threshold for buzzer alert
const int GAS_THRESHOLD = 800;

Servo valveServo; // Servo for valve control

// Function to connect to Wi-Fi
void setup_wifi() {
  delay(10);
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected to Wi-Fi");
}

// Function to publish MQTT message
void publishMessage() {
  float t1 = dhtSensor1.getTemperature();
  float h2 = dhtSensor2.getHumidity();

  if (isnan(t1)) {
    Serial.println("Failed to read from DHT1 sensor. Retrying...");
    t1 = 25.0;
  }
  if (isnan(h2)) {
    Serial.println("Failed to read from DHT2 sensor. Retrying...");
    h2 = 50.0;
  }

  time_t now = time(nullptr);
  String timestamp = String(ctime(&now));

  String payload = "{\"timestamp\":\"" + timestamp + 
                   "\", \"GasConsumption\":\"" + String(gasLevel) + 
                   "\", \"WaterConsumption\":\"" + String(waterConsumptionLevel) + 
                   "\", \"LDR\":\"" + String(ldrValue) + 
                   "\", \"PIR\":\"" + String(pirValue) + 
                   "\", \"DHT1_Temperature\":\"" + String(t1, 2) + 
                   "\", \"DHT2_Humidity\":\"" + String(h2, 1) + "\"}";

  if (client.publish(AWS_IOT_PUBLISH_TOPIC, payload.c_str())) {
    Serial.println("Message published successfully: " + payload);
  } else {
    Serial.println("Message publish failed.");
  }


  Blynk.virtualWrite(V11, pirValue);
  Blynk.virtualWrite(V6, t1);                    // Send temperature to virtual pin V1
  Blynk.virtualWrite(V7, h2);                    // Send humidity to virtual pin V2
  Blynk.virtualWrite(V8, ldrValue);              // Send LDR value to virtual pin V3
  Blynk.virtualWrite(V9, waterConsumptionLevel); // Send water consumption to virtual pin V4
  Blynk.virtualWrite(V10, gasLevel);        
}

void connectAWS() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("Wi-Fi connected!");

  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  client.setServer(AWS_IOT_ENDPOINT, 8883);
  client.setCallback(messageHandler);

  Serial.println("Connecting to AWS IoT...");
  while (!client.connected()) {
    if (client.connect(THINGNAME)) {
      Serial.println("Connected to AWS IoT!");
    } else {
      Serial.print("Connection failed, status: ");
      Serial.println(client.state());
      delay(2000);
    }
  }

  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
}

// MQTT message handler
void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic [");
  Serial.print(topic);
  Serial.print("] : ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Heating/Cooling management
void manageHeatingCooling(float temperature) {
  pirValue = digitalRead(PIR_PIN);

  if (pirValue == HIGH) {
    lastMotionTime = millis();

    if (temperature < 20.0) {
      Serial.println("Temperature is low. Turning on heater.");
      digitalWrite(HEATER_LED_PIN, HIGH);
      digitalWrite(AC_LED_PIN, LOW);
    } else if (temperature > 30.0) {
      Serial.println("Temperature is high. Turning on AC.");
      digitalWrite(HEATER_LED_PIN, LOW);
      digitalWrite(AC_LED_PIN, HIGH);
    } else {
      Serial.println("Temperature is optimal. Turning off heater and AC.");
      digitalWrite(HEATER_LED_PIN, LOW);
      digitalWrite(AC_LED_PIN, LOW);
    }
  } else if (millis() - lastMotionTime > motionTimeout) {
    Serial.println("No motion detected for a while. Turning off heater and AC.");
    digitalWrite(HEATER_LED_PIN, LOW);
    digitalWrite(AC_LED_PIN, LOW);
  }
}

// Lighting management
void manageLighting(int ldrValue) {
  if (ldrValue > 1500) {
    Serial.println("It's bright. Turning off light.");
    digitalWrite(LIGHT_LED_PIN, LOW);  // Turn off light if it's bright enough
  } else  {
    Serial.println("It's dark. Turning on light.");
    digitalWrite(LIGHT_LED_PIN, HIGH); // Turn on light if it's dark
  }
}

// Water leak detection
unsigned long leakStartTime = 0;  // Store the time when water consumption exceeds threshold
const unsigned long LEAK_THRESHOLD_TIME = 3000; // Time threshold for detecting water leak (in milliseconds)

void checkWaterLeak(int waterConsumption) {
  if (waterConsumption > 500) {  // Threshold for water leak
    if (leakStartTime == 0) {  // If this is the first time water consumption is above the threshold
      leakStartTime = millis();  // Record the time when the threshold is first exceeded
    } else if (millis() - leakStartTime > LEAK_THRESHOLD_TIME) {  // If water consumption is high for more than threshold time
      Serial.println("Water leak detected! Turning off valve.");
      digitalWrite(BUZZER_WATER_PIN, HIGH);  // Activate water leak buzzer
      valveServo.write(180);  // Move servo to 180 degrees (valve off)
      digitalWrite(BUZZER_WATER_PIN, LOW);  // Deactivate water leak buzzer
      leakStartTime = 0;  // Reset the timer after the leak has been detected and handled
    }
  } else {
    leakStartTime = 0;  // Reset timer if water consumption falls below the threshold
  }
}

// Smart irrigation
void manageIrrigation(int soilMoisture) {
  if (soilMoisture < 30) {
    Serial.println("Soil moisture is low. Activating irrigation.");
    digitalWrite(IRRIGATION_LED_PIN, HIGH); // Activate irrigation LED
  } else {
    Serial.println("Soil moisture is sufficient. Deactivating irrigation.");
    digitalWrite(IRRIGATION_LED_PIN, LOW);  // Deactivate irrigation LED
  }
}

// Gas monitoring
void monitorGas(int gasLevel) {
  if (gasLevel > GAS_THRESHOLD) {
    Serial.println("Gas level is high! Activating gas buzzer.");
    digitalWrite(BUZZER_GAS_PIN, HIGH); // Gas consumption buzzer
    digitalWrite(BUZZER_GAS_PIN, LOW);
  }
}

BLYNK_WRITE(V_HEATER_PIN) {
  int state = param.asInt();  // Get the value from the Blynk app
  Serial.print("Blynk received heater state: ");
  Serial.println(state);
  digitalWrite(HEATER_LED_PIN, state);  // Turn the heater LED on/off
}

// Function to control AC LED (Virtual pin V2)
BLYNK_WRITE(V_AC_PIN) {
  int state = param.asInt();  // Get the value from the Blynk app
  Serial.print("Blynk received AC state: ");
  Serial.println(state);
  digitalWrite(AC_LED_PIN, state);  // Turn the AC LED on/off
}

// Function to control Light LED (Virtual pin V3)
BLYNK_WRITE(V_LIGHT_PIN) {
  int state = param.asInt();  // Get the value from the Blynk app
  Serial.print("Blynk received light state: ");
  Serial.println(state);
  digitalWrite(LIGHT_LED_PIN, state);  // Turn the light LED on/off
}

// Function to control Irrigation LED (Virtual pin V4)
BLYNK_WRITE(V_IRRIGATION_PIN) {
  int state = param.asInt();  // Get the value from the Blynk app
  Serial.print("Blynk received irrigation state: ");
  Serial.println(state);
  digitalWrite(IRRIGATION_LED_PIN, state);  // Turn the irrigation LED on/off
}

// Function to control Servo (Virtual pin V5)
BLYNK_WRITE(V_SERVO_PIN) {
  int angle = param.asInt();  // Get the angle (0-180) from the Blynk app
  Serial.print("Blynk received servo angle: ");
  Serial.println(angle);
  valveServo.write(angle);  // Move the servo to the specified angle
}

void setup() {
  Serial.begin(115200);
  pinMode(HEATER_LED_PIN, OUTPUT);
  pinMode(AC_LED_PIN, OUTPUT);
  pinMode(LIGHT_LED_PIN, OUTPUT);
  pinMode(IRRIGATION_LED_PIN, OUTPUT);
  pinMode(BUZZER_GAS_PIN, OUTPUT);
  pinMode(BUZZER_WATER_PIN, OUTPUT);

  valveServo.attach(SERVO_PIN);  // Attach servo to control valve

  connectAWS();

  dhtSensor1.setup(DHT1_PIN, DHTesp::DHT22);  // Setup DHT1 for temperature
  dhtSensor2.setup(DHT2_PIN, DHTesp::DHT22);  // Setup DHT2 for soil moisture

  pinMode(PIR_PIN, INPUT);    // PIR sensor input
  pinMode(LDR_PIN, INPUT);    // LDR sensor input
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);
   if (Blynk.connected()) {
    Serial.println("Blynk connected!");
  } else {
    Serial.println("Blynk not connected!");
  }
}


void loop() {
  float t1 = dhtSensor1.getTemperature();
  float h2 = dhtSensor2.getHumidity(); // Soil moisture from DHT2

  manageHeatingCooling(t1);

  gasLevel = analogRead(POTENTIOMETER_1_PIN);
  waterConsumptionLevel = analogRead(POTENTIOMETER_2_PIN);
  soilMoisture = h2; // Use the humidity reading as soil moisture
  ldrValue = analogRead(LDR_PIN);

  manageLighting(ldrValue);
  checkWaterLeak(waterConsumptionLevel);  // Check for water leak and turn off valve if necessary
  manageIrrigation(soilMoisture);
  monitorGas(gasLevel);
  Blynk.run();
  if (!client.connected()) {
    connectAWS();
  }
  client.loop();
 
  delay(5000);
  publishMessage();
  
}
