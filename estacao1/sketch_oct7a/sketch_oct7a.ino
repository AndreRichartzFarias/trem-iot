#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include "env.h"

WiFiClientSecure wifi_client;
PubSubClient mqtt(wifi_client);


const String brokerUser = "";
const String brokerPass = "";


const byte pinLED = 2; // 

void setup() {
  Serial.begin(115200);
  wifi_client.setInsecure();
  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, LOW);
  WiFi.begin(WIFI_SSID, WIFI_PASS);  
  Serial.println("Conectando no WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(200);
  }
  Serial.println("Conectado com sucesso!");

  mqtt.setServer(BROKER_URL.c_str(), BROKER_PORT);
  String clientID = "AndreFarias";
  clientID += String(random(0xffff), HEX);
  while (mqtt.connect(clientID.c_str()) == 0) {
    Serial.print(".");
    delay(2000);
  }
  mqtt.subscribe(TOPIC_PRESENCE1.c_str());
  mqtt.setCallback(callback);
  Serial.println("\nConectado ao broker!");
}

void loop() {
  mqtt.loop();

  // Ler dados da Serial e enviar
  if (Serial.available() > 0) {               
    String mensagem = Serial.readStringUntil('\n');  
    mensagem.trim(); // Remove quebras de linha extras
    // Lógica para ligar/desligar o LED da dupla enviando uma mensagem em números true/false
    if (mensagem == "1") {
      digitalWrite(pinLED, HIGH);
      Serial.println("LED LIGADO (comando serial)");
      mqtt.publish(TOPIC_PRESENCE1.c_str(), "LED ON");
    } 
    else if (mensagem == "0") { 
      digitalWrite(pinLED, LOW);
      Serial.println("LED DESLIGADO (comando serial)");
      mqtt.publish(TOPIC_PRESENCE1.c_str(), "LED OFF");
    } 
    else {
      String texto = "Andre: " + mensagem;
      mqtt.publish("LauraKrueger", texto.c_str());
      Serial.print("Mensagem enviada: ");
      Serial.println(texto);
    }
  }
}



void callback(char* topic, byte* payload, unsigned long length) {
  String mensagem_recebida = "";
  for(int i=0; i< length; i++){
    mensagem_recebida += (char) payload[i]; // Agrupa cada caracter recebida func payload[variavel]
  }
  Serial.println(mensagem_recebida); // Colocar o LED Pino2
}


void connecttoBroker(){
  Serial.println("Conectando ao broker...");
}