#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient wifi_client;
PubSubClient mqtt(wifi_client);
const String SSID = "FIESC_IOT_EDU";
const String PASS = "8120gv08";

const String brokerURL = "test.mosquitto.org";
const int brokerPort = 1883;

const String brokerUser = "";
const String brokerPass = "";

void setup() {
  Serial.begin(115200);
  client.begin(SSID, PASS);
  Serial.println("Conectado no Wifi");
  while(cliente.status() != WL_CONNECTED){
    Serial.print(".");
    delay(200);
  }
  Serial.println("Conectado com sucesso!");
  mqtt.setServer(brokerURL.c_str(),brokerPort);
  String clientID = "estacao1-";
  clientID += String(random(0xffff), HEX);
  Serial.prinln("Conectando ao Broker!");
  while(mqtt.connect(clienteID.c_str()) == 0) {
    Serial.println(".");
    delay(200);
  }
  Serial.println("Conectado ao broker!");
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}