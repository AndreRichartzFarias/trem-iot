#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "Ultrasonic.h"
#include "env.h"
#include <ESP32Servo.h>

// --- Pin definitions (adjust if you wire differently) ---
#define ULTRA_TRIG_PIN_1 22
#define ULTRA_ECHO_PIN_1 23
#define SERVO_PIN_1 27
#define SERVO_PIN_2 33
#define LED_YELLOW_PIN 19

// --- Parameters ---
#define LOCAL_PRESENCE_DISTANCE_CM 10
#define LOCAL_PRESENCE_DEBOUNCE_MS 1000

// --- Objects ---
Ultrasonic ultrasonic(ULTRA_TRIG_PIN_1, ULTRA_ECHO_PIN_1);
Servo servo1;
Servo servo2;
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// --- State ---
static bool lastP1 = false; // last seen state from Station S2 presence1
static bool lastP2 = false; // last seen state from Station S2 presence2
static bool localPresence = false; // current presence from local ultrasonic
static unsigned long lastLocalPublish = 0;

// servo semantic angles: -45 left, 0 center, 45 right
static int servo1Angle = 0;
static int servo2Angle = 0;

// --- Helpers ---
void writeServo(Servo &s, int semanticAngle) {
    // semanticAngle is -45..45, map to 45..135 degrees for servo
    int deg = map(semanticAngle, -45, 45, 45, 135);
    s.write(deg);
}

void publishIfConnected(const char* topic, const char* payload) {
    if (mqttClient.connected()) {
        mqttClient.publish(topic, payload);
        Serial.print("Published "); Serial.print(topic); Serial.print(" => "); Serial.println(payload);
    }
}

void updateServo1FromPresence() {
    // Decide servo1Angle from lastP1 and lastP2
    if (lastP1 && !lastP2) servo1Angle = -45; // left
    else if (lastP2 && !lastP1) servo1Angle = 45; // right
    else servo1Angle = 0; // center
    writeServo(servo1, servo1Angle);
}

void updateServo2FromPresence(bool presence) {
    servo2Angle = presence ? -45 : 0;
    writeServo(servo2, servo2Angle);
}

// --- MQTT callback ---
void callback(char* topic, byte* payload, unsigned int length) {
    String msg;
    for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
    Serial.print("MQTT RX: "); Serial.print(topic); Serial.print(" => "); Serial.println(msg);

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

    // Local presence topic (so servo2 can also be controlled remotely or by local publications)
    if (strcmp(topic, topicPresenceSensorLocal) == 0) {
        bool p = (msg == "1");
        updateServo2FromPresence(p);
        return;
    }

    // Luminance topic controls LED
    if (strcmp(topic, topicLuminanceSensor) == 0) {
        if (msg == "1") digitalWrite(LED_YELLOW_PIN, HIGH);
        else digitalWrite(LED_YELLOW_PIN, LOW);
        return;
    }
}

// --- WiFi / MQTT management ---
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
        if (mqttClient.connect(clientID.c_str(), MQTT_USER_CONN, MQTT_PASSWORD_CONN)) {
            Serial.println(" connected");
            // subscribe to controlling topics
            mqttClient.subscribe(topicPresenceSensor2_1);
            mqttClient.subscribe(topicPresenceSensor2_2);
            mqttClient.subscribe(topicLuminanceSensor);
            // subscribe to local presence topic so servo2 can follow remote/local updates
            mqttClient.subscribe(topicPresenceSensorLocal);
            return;
        }
        Serial.print('.');
        delay(1000);
    }
    Serial.println(" MQTT connect failed");
}

// --- Setup / Loop ---
void setup() {
    Serial.begin(115200);
    wifiClient.setInsecure(); // HiveMQ free tier without CA

    // Servo initialization
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

void loop() {
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    if (!mqttClient.connected()) connectMQTT();
    mqttClient.loop();

    // Read ultrasonic periodically and publish local presence when it changes (debounced)
    float distance = ultrasonic.read(); // cm or <=0
    bool present = (distance > 0 && distance < LOCAL_PRESENCE_DISTANCE_CM);
    unsigned long now = millis();
    if (present != localPresence && (now - lastLocalPublish) > LOCAL_PRESENCE_DEBOUNCE_MS) {
        localPresence = present;
        // publish local presence to topicPresenceSensorLocal
        publishIfConnected(topicPresenceSensorLocal, localPresence ? "1" : "0");
        lastLocalPublish = now;
        // update servo2 locally as well
        updateServo2FromPresence(localPresence);
    }

    delay(100);
}
