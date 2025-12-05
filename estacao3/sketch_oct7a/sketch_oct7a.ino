#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "Ultrasonic.h"
#include "env.h"
#include <ESP32Servo.h>

// Pin definitions
#define ULTRA_TRIG_PIN_1 22
#define ULTRA_ECHO_PIN_1 23
#define SERVO_PIN_1 27
#define SERVO_PIN_2 33
#define LED_YELLOW_PIN 19

// Parameters
#define LOCAL_PRESENCE_DISTANCE_CM 10
#define LOCAL_PRESENCE_DEBOUNCE_MS 1000

// Objects
Ultrasonic ultrasonic(ULTRA_TRIG_PIN_1, ULTRA_ECHO_PIN_1);
Servo servo1, servo2;
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

//State variables
static bool lastP1 = false;
static bool lastP2 = false;
static bool localPresence = false;
static unsigned long lastLocalPublish = 0;

static int servo1Angle = 0;
static int servo2Angle = 0;

// Servo helper
// Converts semantic angles (-45..45) to servo degrees (45..135)
void writeServo(Servo &s, int semanticAngle) {
    int deg = map(semanticAngle, -45, 45, 45, 135);
    s.write(deg);
}

// MQTT publish helper
void publishIfConnected(const char* topic, const char* payload) {
    if (mqttClient.connected()) {
        mqttClient.publish(topic, payload);
        Serial.printf("Published %s => %s\n", topic, payload);
    }
}

// Servo logic
// Updates servo1 based on presence from remote station
void updateServo1FromPresence() {
    if (lastP1 && !lastP2)      servo1Angle = -45;
    else if (lastP2 && !lastP1) servo1Angle = 45;
    else                        servo1Angle = 0;

    writeServo(servo1, servo1Angle);
}

// Updates servo2 based on local or remote presence signal
void updateServo2FromPresence(bool presence) {
    servo2Angle = presence ? -45 : 0;
    writeServo(servo2, servo2Angle);
}

// MQTT callback
void callback(char* topic, byte* payload, unsigned int length) {
    String msg;
    for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

    Serial.printf("MQTT RX: %s => %s\n", topic, msg.c_str());

    if (strcmp(topic, topicPresenceSensor2_1) == 0) {
        lastP1 = (msg == "1");
        updateServo1FromPresence();
        return;
    }

    if (strcmp(topic, topicPresenceSensor2_2) == 0) {
        lastP2 = (msg == "1");
        updateServo1FromPresence();
        return;
    }

    if (strcmp(topic, topicPresenceSensorLocal) == 0) {
        updateServo2FromPresence(msg == "1");
        return;
    }

    if (strcmp(topic, topicLuminanceSensor) == 0) {
        digitalWrite(LED_YELLOW_PIN, msg == "1" ? HIGH : LOW);
        return;
    }
}

// WiFi / MQTT connect
void connectWiFi() {
    Serial.print("Connecting WiFi...");
    WiFi.begin(WIFI_CONN_SSID, WIFI_CONN_PASSWORD);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        Serial.print('.');
        delay(500);
    }
    Serial.println(WiFi.status() == WL_CONNECTED ? " connected" : " failed");
}

void connectMQTT() {
    if (WiFi.status() != WL_CONNECTED) return;

    Serial.print("Connecting MQTT...");
    String clientID = "EST3-" + String(random(0xffff), HEX);

    unsigned long start = millis();
    while (!mqttClient.connected() && millis() - start < 15000) {
        if (mqttClient.connect(clientID.c_str(), BROKER_USR_ID, BROKER_USR_PASS)) {
            Serial.println(" connected");
            mqttClient.subscribe(topicPresenceSensor2_1);
            mqttClient.subscribe(topicPresenceSensor2_2);
            mqttClient.subscribe(topicLuminanceSensor);
            mqttClient.subscribe(topicPresenceSensorLocal);
            return;
        }
        Serial.print('.');
        delay(1000);
    }
    Serial.println(" MQTT connect failed");
}

// Setup
void setup() {
    Serial.begin(115200);
    wifiClient.setInsecure();

    servo1.attach(SERVO_PIN_1);
    servo2.attach(SERVO_PIN_2);

    updateServo1FromPresence();
    updateServo2FromPresence(false);

    pinMode(LED_YELLOW_PIN, OUTPUT);
    digitalWrite(LED_YELLOW_PIN, LOW);

    connectWiFi();
    mqttClient.setServer(MQTT_BROKER_CONN, MQTT_PORT_CONN);
    mqttClient.setCallback(callback);
    connectMQTT();
}

// Main loop
void loop() {
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    if (!mqttClient.connected()) connectMQTT();
    mqttClient.loop();

    // Handle local ultrasonic presence
    float distance = ultrasonic.read();
    bool present = (distance > 0 && distance < LOCAL_PRESENCE_DISTANCE_CM);

    unsigned long now = millis();
    if (present != localPresence && now - lastLocalPublish > LOCAL_PRESENCE_DEBOUNCE_MS) {
        localPresence = present;
        publishIfConnected(topicPresenceSensorLocal, present ? "1" : "0");
        lastLocalPublish = now;
        updateServo2FromPresence(present);
    }

    delay(100);
}
