#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "Ultrasonic.h"
#include "env.h"


#define ULTRA_TRIG_PIN_1 22
#define ULTRA_ECHO_PIN_1 23
#define ULTRA_TRIG_PIN_2 27
#define ULTRA_ECHO_PIN_2 33
#define LED_YELLOW_PIN 19


#define PRESENCE_THRESHOLD 20.0  // cm
#define SENSOR_READ_INTERVAL 5000 // ms

Ultrasonic ultrasonic1(ULTRA_TRIG_PIN_1, ULTRA_ECHO_PIN_1);
Ultrasonic ultrasonic2(ULTRA_TRIG_PIN_2, ULTRA_ECHO_PIN_2);
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

struct SensorState {
    unsigned long lastPresenceChange = 0;
    bool presenceDetected = false;
};
SensorState sensors;

void setupHardware();
void connectWiFi();
void connectMQTT();
void readSensors();
void publishData(const char* topic, const String& value);
void callback(char* topic, byte* payload, unsigned int length);

void setup() {
    Serial.begin(115200);
    Serial.println("=== RailFlow Station S2 Starting ===");
    setupHardware();
    connectWiFi();
    mqttClient.setServer(BROKER_URL, BROKER_PORT);
    mqttClient.setCallback(callback);
    connectMQTT();
    publishData("RailFlow/S2/Debug", "Sistema inicializado");
    Serial.println("=== System Ready ===");
}

void setupHardware() {
    pinMode(LED_YELLOW_PIN, OUTPUT);
    digitalWrite(LED_YELLOW_PIN, LOW);
}

void publishData(const char* topic, const String& value) {
    if (mqttClient.connected()) {
        mqttClient.publish(topic, value.c_str());
        Serial.println("Published: " + String(topic) + " = " + value);
    }
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        connectWiFi();
    }
    if (!mqttClient.connected()) {
        connectMQTT();
    }
    mqttClient.loop();
    readSensors();
    delay(100);
}

void readSensors() {
    float distance1 = ultrasonic1.read();
    float distance2 = ultrasonic2.read();
    bool presence1 = (distance1 < PRESENCE_THRESHOLD);
    bool presence2 = (distance2 < PRESENCE_THRESHOLD);
    static bool lastPresence1 = false;
    static bool lastPresence2 = false;
    // Sensor 1
    if (presence1 != lastPresence1) {
        lastPresence1 = presence1;
        publishData(topicPresenceSensor2_1, presence1 ? "1" : "0");
    }
    // Sensor 2
    if (presence2 != lastPresence2) {
        lastPresence2 = presence2;
        publishData(topicPresenceSensor2_2, presence2 ? "1" : "0");
    }
}

void connectWiFi() {
    Serial.print("Connecting to WiFi...");
    wifiClient.setInsecure();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 20000) {
        Serial.print(".");
        delay(500);
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" Connected!");
        Serial.println("IP: " + WiFi.localIP().toString());
    } else {
        Serial.println(" Failed!");
    }
}

void connectMQTT() {
    if (WiFi.status() != WL_CONNECTED) return;
    Serial.print("Connecting to MQTT...");
    String clientID = "RailFlow-S2-" + String(random(0xffff), HEX);
    unsigned long startTime = millis();
    while (!mqttClient.connected() && millis() - startTime < 15000) {
        if (mqttClient.connect(clientID.c_str(), BROKER_USR_ID, BROKER_USR_PASS)) {
            Serial.println(" Connected!");
            mqttClient.subscribe(topicLuminanceControl);
            return;
        }
        delay(2000);
    }
    if (!mqttClient.connected()) {
        Serial.println(" Failed!");
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.println("MQTT: " + String(topic) + " = " + message);
    if (String(topic) == topicLuminanceControl) {
        if (message == "1") {
            digitalWrite(LED_YELLOW_PIN, HIGH);
        } else {
            digitalWrite(LED_YELLOW_PIN, LOW);
        }
    }
}