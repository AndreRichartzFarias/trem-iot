#ifndef ENV_H
#define ENV_H

// ---Private WiFi configuration---
#define WIFI_SSID "FIESC_IOT_EDU"
#define WIFI_PASS "8120gv08"

// ---Broker configuration---
#define BROKER_URL "36e8a0cdfccc4299a8cca62d05fc37c8.s1.eu.hivemq.cloud" // ---URL Sensitive Data---
#define BROKER_PORT 8883
#define BROKER_USR_ID "AndreFarias" // ---Sensor ID 1---
#define BROKER_USR_PASS "AndreFarias1" // ---Sensors Password---


#define WIFI_CONN_SSID WIFI_SSID
#define WIFI_CONN_PASSWORD WIFI_PASS

// --- Station S1 topics  ---
#define topicTemperatureSensor "RailFlow/S1/Temp"
#define topicHumiditySensor    "RailFlow/S1/Umid"
#define topicLuminanceSensor   "RailFlow/S1/Lum"
#define topicPresenceSensor1   "RailFlow/S1/Presenca1"

#endif
