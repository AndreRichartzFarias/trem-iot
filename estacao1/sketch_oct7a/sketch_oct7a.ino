#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient wifi_client;
PubSubClient mqtt(wifi_client);

const char* SSID = "FIESC_IOT_EDU";
const char* PASS = "8120gv08";

const char* brokerURL = "test.mosquitto.org";
const int brokerPort = 1883;
const String topic = "Andrezao";

const char* brokerUser = "";
const char* brokerPass = "";

int ledPin = 2;

void setup() {
  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);

  // Conecta ao Wi-Fi
  WiFi.begin(SSID, PASS);
  Serial.print("Conectando ao Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nConectado com sucesso ao Wi-Fi!");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());

  // Conecta ao broker MQTT
  mqtt.setServer(brokerURL, brokerPort);
  String clientID = "estacao1-";
  clientID += String(random(0xffff), HEX);

  Serial.println("Conectando ao Broker MQTT...");
  while (!mqtt.connect(clientID.c_str(), brokerUser, brokerPass)) {
    Serial.print(".");
    delay(500);
  }

  mqtt.subscribe(topic.c_str());
  mqtt.setCallback(callback);
  Serial.println("\nConectado ao broker MQTT!");
}

void loop() {
  // Mantém a conexão MQTT viva
  if (!mqtt.connected()) {
    String clientID = "estacao1-";
    clientID += String(random(0xffff), HEX);
    mqtt.connect(clientID.c_str(), brokerUser, brokerPass);
  }
  mqtt.loop();

  // Lê mensagens do Serial
  String mensagem = "";
  if (Serial.available() > 0) {
    mensagem = Serial.readStringUntil('\n');
    mensagem = "André: " + mensagem;
    Serial.println(mensagem);

    // Publica no tópico MQTT
    mqtt.publish("churros", mensagem.c_str());
    Serial.println("Mensagem publicada no broker!");
  }
  mqtt.loop();
}

void callback(char* topic, byte* payload, unsigned long length) {
  String MensagemRecebida = "";
  for (int i = 0; i < length; i++) {
    // pega letras do payload e une
    MensagemRecebida += (char)payload[i];
  }

  Serial.println(MensagemRecebida);

  if (MensagemRecebida.indexOf("acender") != -1) {
    digitalWrite(ledPin, HIGH);
  }
}
