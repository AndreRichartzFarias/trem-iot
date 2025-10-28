#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include "env.h"

WiFiClientSecure wifi_client;
PubSubClient mqtt(wifi_client);

// Debug function to send messages to RailFlow/S1/Temp1
void sendDebugMessage(String message) {
  String debugMsg = "DEBUG: " + message;
  mqtt.publish(TOPIC_DEBUG, debugMsg.c_str());
  Serial.print("Debug message sent to ");
  Serial.print(TOPIC_DEBUG);
  Serial.print(": ");
  Serial.println(debugMsg);
}

void setup() {
  Serial.begin(115200);
  wifi_client.setInsecure();
  
  WiFi.begin(WIFI_SSID, WIFI_PASS);  
  Serial.println("Conectando no WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(200);
  }
  Serial.println("Conectado com sucesso!");
  mqtt.setServer(BROKER_URL, BROKER_PORT);
  String clientID = "AndreFarias";
  clientID += String(random(0xffff), HEX);
  
  Serial.println("Conectando ao broker MQTT...");
  while (!mqtt.connect(clientID.c_str(), BROKER_USR_ID, BROKER_USR_PASS)) {
    Serial.print("Falha na conexão MQTT, código: ");
    Serial.print(mqtt.state());
    Serial.println(" - Tentando novamente em 5 segundos...");
    delay(5000);
  }
  mqtt.setCallback(callback);
  Serial.println("\nConectado ao broker!");
  
  // Send initial debug message
  sendDebugMessage("Sistema inicializado com sucesso");
}

void loop() {
  mqtt.loop();

  // Send periodic debug message every 30 seconds
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 30000) {
    sendDebugMessage("Sistema funcionando - Uptime: " + String(millis()/1000) + "s");
    lastDebugTime = millis();
  }

  // Check for debug command via Serial
  if (Serial.available() > 0) {               
    String mensagem = Serial.readStringUntil('\n');  
    mensagem.trim();
    
    if (mensagem.startsWith("debug:")) {
      String debugText = mensagem.substring(6); // Remove "debug:" prefix
      sendDebugMessage(debugText);
    }
  }
}



void callback(char* topic, byte* payload, unsigned long length) {
  String mensagem_recebida = "";
  for(int i=0; i< length; i++){
    mensagem_recebida += (char) payload[i];
  }
  Serial.println("Mensagem recebida: " + mensagem_recebida);
}