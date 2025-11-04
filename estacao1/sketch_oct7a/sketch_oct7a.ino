#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include "env.h"

// Pinout and hardware mapping (adjust pins to your board before flashing)
#define LDR_PIN 34           // ADC input for LDR (ADC1_CH6)
#define DHT_PIN 27           // DHT data pin
#define ULTRA_1_TRIGG 12     // Ultrasonic sensor TRIG
#define ULTRA_1_ECHO 13      // Ultrasonic sensor ECHO

// Status RGB LED (using LEDC PWM on ESP32)
#define STATUS_LED_R_PIN 15
#define STATUS_LED_G_PIN 2
#define STATUS_LED_B_PIN 4
#define PWM_CHANNEL_LED_R 0
#define PWM_CHANNEL_LED_G 1
#define PWM_CHANNEL_LED_B 2
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// Simple digital LED pin (optional)
#define LEDPIN 5

// Peripherals placeholders (no LED or extra logic per user request)
#include "DHT.h"
#include "Ultrasonic.h"

// Sensors
Ultrasonic ultrasonic(ULTRA_1_TRIGG, ULTRA_1_ECHO);
DHT dht(DHT_PIN, DHT11);

WiFiClientSecure client;
PubSubClient mqttClient(client);

// Timing
unsigned long lastPresenceDetection = 0;
unsigned long lastLightReading = 0;
unsigned long lastTempReading = 0;

bool detected = false;
bool lastLightStatus = false;
byte luminanceThreshold = 30;

void callback(char* topic, byte* payload, unsigned int length);
void connectToWiFi();
void connectToMQTT();
void sendDebugMessage(const String &message);
void publishSensor(const char* topic, const String &payload);
void publishSensor(const char* topic, const char* payload);

void sendDebugMessage(const String &message) {
    // Dedicated debug function for initial tests only
    String debugMsg = "DEBUG: " + message;
    mqttClient.publish(topicDebug, debugMsg.c_str());
    Serial.print("Debug message sent to ");
    Serial.print(topicDebug);
    Serial.print(": ");
    Serial.println(debugMsg);
}

void publishSensor(const char* topic, const String &payload) {
    mqttClient.publish(topic, payload.c_str());
    Serial.print("Published to ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(payload);
}

void publishSensor(const char* topic, const char* payload) {
    mqttClient.publish(topic, payload);
    Serial.print("Published to ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(payload);
}

void setup() {
    Serial.begin(115200);
    client.setInsecure(); // HiveMQ free tier

    // Initialize status LED PWM channels and attach pins
    ledcSetup(PWM_CHANNEL_LED_R, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_LED_G, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_LED_B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(STATUS_LED_R_PIN, PWM_CHANNEL_LED_R);
    ledcAttachPin(STATUS_LED_G_PIN, PWM_CHANNEL_LED_G);
    ledcAttachPin(STATUS_LED_B_PIN, PWM_CHANNEL_LED_B);
    // Turn off LEDs initially
    ledcWrite(PWM_CHANNEL_LED_R, 0);
    ledcWrite(PWM_CHANNEL_LED_G, 0);
    ledcWrite(PWM_CHANNEL_LED_B, 0);

    // Simple digital LED
    pinMode(LEDPIN, OUTPUT);
    digitalWrite(LEDPIN, LOW);

    // Initialize sensors
    dht.begin();

    // Start WiFi and MQTT
    connectToWiFi();
    mqttClient.setServer(MQTT_BROKER_CONN, MQTT_PORT_CONN);
    mqttClient.setCallback(callback);
    connectToMQTT();

    // one-time debug test publish
    sendDebugMessage("Sistema inicializado com sucesso");
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        connectToWiFi();
    }
    if (!mqttClient.connected()) {
        connectToMQTT();
    }
    mqttClient.loop();

    unsigned long currentTime = millis();

    // Luminance
    if (currentTime - lastLightReading > 5000) {
        lastLightReading = currentTime;
        byte luminanceValue = map(analogRead(LDR_PIN), 0, 4095, 0, 100);
        Serial.println("Luminance: " + String(luminanceValue));
        if (luminanceValue < luminanceThreshold && !lastLightStatus) {
            lastLightStatus = true;
            publishSensor(topicLuminanceSensor, "1");
        } else if (luminanceValue >= luminanceThreshold && lastLightStatus) {
            lastLightStatus = false;
            publishSensor(topicLuminanceSensor, "0");
        }
    }

    // Temperature/Humidity
    if (currentTime - lastTempReading > 3000) {
        lastTempReading = currentTime;
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        if (!isnan(temperature) && !isnan(humidity)) {
            publishSensor(topicTemperatureSensor, String(temperature));
            publishSensor(topicHumiditySensor, String(humidity));
        } else {
            Serial.println("DHT read error");
        }
    }

    // Presence (ultrasonic)
    long microsec = ultrasonic.timing();
    float distance = ultrasonic.convert(microsec, Ultrasonic::CM);
    if (distance < 10 && !detected && (currentTime - lastPresenceDetection >= 1000)) {
        publishSensor(topicPresenceSensor1, "1");
        detected = true;
        lastPresenceDetection = currentTime;
    }
    if (distance >= 10 && detected && (currentTime - lastPresenceDetection >= 1000)) {
        publishSensor(topicPresenceSensor1, "0");
        detected = false;
        lastPresenceDetection = currentTime;
    }

    delay(100);
}

void connectToWiFi() {
    Serial.print("Connecting to WiFi ");
    Serial.println(WIFI_CONN_SSID);
    WiFi.begin(WIFI_CONN_SSID, WIFI_CONN_PASSWORD);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
        Serial.print(".");
        delay(500);
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected");
    } else {
        Serial.println("\nWiFi connection failed");
    }
}

void connectToMQTT() {
    Serial.print("Connecting to MQTT ");
    Serial.println(MQTT_BROKER_CONN);

    String clientID = "AndreFarias-";
    clientID += String(random(0xffff), HEX);

    unsigned long start = millis();
    while (!mqttClient.connected() && millis() - start < 20000) {
        Serial.print("Attempting MQTT connection...");
        if (mqttClient.connect(clientID.c_str(), MQTT_USER_CONN, MQTT_PASSWORD_CONN)) {
            Serial.println("connected");
            mqttClient.subscribe(topicLuminanceSensor);
            mqttClient.subscribe(topicPresenceSensor1);
            return;
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 2 seconds");
            delay(2000);
        }
    }
    if (!mqttClient.connected()) {
        Serial.println("Could not connect to MQTT broker within timeout");
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    String mensagem = "";
    for (unsigned int i = 0; i < length; i++) {
        mensagem += (char)payload[i];
    }
    Serial.print("Received on ");
    Serial.print(topic);
    Serial.print(" : ");
    Serial.println(mensagem);
}