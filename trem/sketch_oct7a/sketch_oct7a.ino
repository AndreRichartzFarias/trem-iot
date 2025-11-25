#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "env.h"

// --- Pinos dos LEDs indicadores ---
#define LED_FRENTE 21     // LED verde (frente)
#define LED_RE 18         // LED vermelho (ré)

// --- Objetos de comunicação ---
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// --- Função de callback MQTT ---
void callback(char* topic, byte* payload, unsigned long length);

// --- Conexão ao broker MQTT ---
void connectMQTT();
void connectWiFi();

void setup() {
    Serial.begin(115200);
    delay(500);

    // Inicializa LEDs
    pinMode(LED_FRENTE, OUTPUT);
    pinMode(LED_RE, OUTPUT);
    digitalWrite(LED_FRENTE, LOW);
    digitalWrite(LED_RE, LOW);

    // Conecta ao WiFi
    connectWiFi();

    // Configura MQTT
    mqttClient.setServer(BROKER_URL, BROKER_PORT);
    mqttClient.setCallback(callback);
    connectMQTT();
}

void loop() {
    // Garante WiFi sempre conectado
    if (WiFi.status() != WL_CONNECTED) {
        connectWiFi();
    }
    // Garante MQTT sempre conectado
    if (!mqttClient.connected()) {
        connectMQTT();
    }
    mqttClient.loop();
}

// --- Conecta ao WiFi ---
void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) return;
    Serial.print("Reconectando ao WiFi...");
    wifiClient.setInsecure();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 20000) {
        Serial.print(".");
        delay(500);
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" Conectado!");
    } else {
        Serial.println(" Falhou!");
    }
}

// --- Conecta e inscreve no tópico MQTT ---
void connectMQTT() {
    if (WiFi.status() != WL_CONNECTED) return;
    Serial.print("Conectando ao broker...");
    while (!mqttClient.connected()) {
        if (mqttClient.connect("RailFlow-S2-Trem", BROKER_USR_ID, BROKER_USR_PASS)) {
            Serial.println("\nConectado ao MQTT!");
            mqttClient.subscribe(TOPIC_TREMVEL_S2);
            Serial.print("Inscrito no tópico: ");
            Serial.println(TOPIC_TREMVEL_S2);
        } else {
            Serial.print(".");
            delay(2000);
        }
    }
}

// --- Interpreta comando recebido e indica estado ---
void callback(char* topic, byte* payload, unsigned long length) {
    String msg = "";
    for (int i = 0; i < length; i++) msg += (char)payload[i];
    msg.trim();
    int valor = msg.toInt();

    Serial.print("Valor recebido MQTT: ");
    Serial.println(valor);

    if (valor > 0) {
        // Trem indo para frente
        digitalWrite(LED_FRENTE, HIGH);
        digitalWrite(LED_RE, LOW);
        Serial.printf("Indicador: FRENTE | Velocidade: %d\n", valor);
    } else if (valor < 0) {
        // Trem indo para trás
        digitalWrite(LED_FRENTE, LOW);
        digitalWrite(LED_RE, HIGH);
        Serial.printf("Indicador: RÉ | Velocidade: %d\n", abs(valor));
    } else {
        // Trem parado
        digitalWrite(LED_FRENTE, LOW);
        digitalWrite(LED_RE, LOW);
        Serial.println("Indicador: PARADO");
    }
}