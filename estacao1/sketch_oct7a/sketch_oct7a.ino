#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "DHT.h"
#include "Ultrasonic.h"
#include "env.h"

// Hardware pin definitions
#define LDR_PIN 34
#define DHT_PIN 4
#define ULTRA_TRIG_PIN 22
#define ULTRA_ECHO_PIN 23
#define LED_PIN 19
#define LED_R_PIN 14
#define LED_G_PIN 26
#define LED_B_PIN 25

// Configuration constants
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define PRESENCE_THRESHOLD 50.0  // cm
#define LIGHT_THRESHOLD 30       // 0-100 scale
#define SENSOR_READ_INTERVAL 5000 // ms

// Global objects
Ultrasonic ultrasonic(ULTRA_TRIG_PIN, ULTRA_ECHO_PIN);
DHT dht(DHT_PIN, DHT11); 
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// State variables
struct SensorState {
    unsigned long lastTempRead = 0;
    unsigned long lastLightRead = 0;
    unsigned long lastPresenceChange = 0;
    bool presenceDetected = false;
    bool lightState = false;
};

SensorState sensors;

// Function declarations
void setupHardware();
void connectWiFi();
void connectMQTT();
void readSensors();
void publishData(const char* topic, const String& value);
void callback(char* topic, byte* payload, unsigned int length);

void setup() {
    Serial.begin(115200);
    Serial.println("=== RailFlow Station S1 Starting ===");
    
    setupHardware();
    
    // Initialize sensors
    dht.begin();
    delay(2000);
    Serial.println("Sensors initialized");
    
    // Connect to network and MQTT
    connectWiFi();
    mqttClient.setServer(MQTT_BROKER_CONN, MQTT_PORT_CONN);
    mqttClient.setCallback(callback);
    connectMQTT();
    
    publishData(topicDebug, "Sistema inicializado");
    Serial.println("=== System Ready ===");
}

void setupHardware() {
    // Configure RGB LED pins
    ledcAttach(LED_R_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(LED_G_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(LED_B_PIN, PWM_FREQ, PWM_RESOLUTION);
    
    // Turn off LEDs initially
    ledcWrite(LED_R_PIN, 0);
    ledcWrite(LED_G_PIN, 0);
    ledcWrite(LED_B_PIN, 0);
    
    // Configure digital LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}

void publishData(const char* topic, const String& value) {
    if (mqttClient.connected()) {
        mqttClient.publish(topic, value.c_str());
        Serial.println("Published: " + String(topic) + " = " + value);
    }
}

void loop() {
    // Maintain connections
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
    unsigned long currentTime = millis();
      // Read temperature and humidity
    if (currentTime - sensors.lastTempRead > SENSOR_READ_INTERVAL) {
        sensors.lastTempRead = currentTime;
        
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        
        publishData(topicTemperatureSensor, String(temperature, 1));
        publishData(topicHumiditySensor, String(humidity, 1));
    }
    
    // Read light sensor
    if (currentTime - sensors.lastLightRead > SENSOR_READ_INTERVAL) {
        sensors.lastLightRead = currentTime;
        
        int rawValue = analogRead(LDR_PIN);
        int lightLevel = map(rawValue, 0, 4095, 0, 100);
        
        publishData(topicLuminanceSensor, String(lightLevel));
        
        // Update light state for local logic
        sensors.lightState = (lightLevel < LIGHT_THRESHOLD);
    }
    
    // Read ultrasonic sensor and check presence
    float distance = ultrasonic.read();
    
    // Publish distance every 3 seconds
    static unsigned long lastDistancePublish = 0;
    if (currentTime - lastDistancePublish > 3000) {
        lastDistancePublish = currentTime;
        publishData("RailFlow/S1/Distance", String(distance, 1));
    }
    
    // Check for presence changes
    bool currentPresence = (distance < PRESENCE_THRESHOLD);
    
    if (currentPresence != sensors.presenceDetected && 
        (currentTime - sensors.lastPresenceChange > 2000)) {
        
        sensors.presenceDetected = currentPresence;
        sensors.lastPresenceChange = currentTime;
        
        publishData(topicPresenceSensor1, currentPresence ? "1" : "0");
    }
}

void connectWiFi() {
    Serial.print("Connecting to WiFi...");
    wifiClient.setInsecure();
    WiFi.begin(WIFI_CONN_SSID, WIFI_CONN_PASSWORD);
    
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
    String clientID = "RailFlow-S1-" + String(random(0xffff), HEX);
    
    unsigned long startTime = millis();
    while (!mqttClient.connected() && millis() - startTime < 15000) {
        if (mqttClient.connect(clientID.c_str(), MQTT_USER_CONN, MQTT_PASSWORD_CONN)) {
            Serial.println(" Connected!");
            
            // Subscribe to relevant topics
            mqttClient.subscribe(topicLuminanceSensor);
            mqttClient.subscribe(topicPresenceSensor1);
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
}