#ifndef ENV_H
#define ENV_H

// ---Environment configuration header file---
// ---Private WiFi configuration---
#define WIFI_SSID "FIESC_IOT_EDU"
#define WIFI_PASS "8120gv08"

// ---Broker configuration---
#define BROKER_URL "36e8a0cdfccc4299a8cca62d05fc37c8.s1.eu.hivemq.cloud" // ---URL Sensitive Data---
#define BROKER_PORT 8883

// ---Broker user credentials depending on the sensor---
#define BROKER_USR_ID "LauraKrueger" // ---Sensor ID 1---
#define BROKER_USR_PASS "LauraKrueger1" // ---Sensors Password--- 

#define topicPresenceSensor2_1 "RailFlow/S2/Presenca1"
#define topicPresenceSensor2_2 "RailFlow/S2/Presenca2"
#define topicLuminanceControl   "RailFlow/S1/Lum"

#endif