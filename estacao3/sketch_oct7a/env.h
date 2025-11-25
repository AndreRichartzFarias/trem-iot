#ifndef ENV_H
#define ENV_H

// --- Environment configuration ---
#define WIFI_SSID "FIESC_IOT_EDU"
#define WIFI_PASS "8120gv08"

#define BROKER_URL "36e8a0cdfccc4299a8cca62d05fc37c8.s1.eu.hivemq.cloud"
#define BROKER_PORT 8883

#define BROKER_USR_ID "AndreFarias"
#define BROKER_USR_PASS "AndreFarias1"

// Topic mappings used by this node and others in the topology
#define topicTemperatureSensor "RailFlow/S1/Temp"
#define topicHumiditySensor    "RailFlow/S1/Umid"
#define topicLuminanceSensor   "RailFlow/S1/Lum"

// Presence topics from Station S2 (this node subscribes to control servo1)
#define topicPresenceSensor2_1 "RailFlow/S2/Presenca1"
#define topicPresenceSensor2_2 "RailFlow/S2/Presenca2"

// Local presence topic for this station (optional)
#define topicPresenceSensorLocal "RailFlow/S3/Presenca1"

// Backwards compatibility names used in sketches
#define MQTT_BROKER_CONN BROKER_URL
#define MQTT_PORT_CONN BROKER_PORT
#define MQTT_USER_CONN BROKER_USR_ID
#define MQTT_PASSWORD_CONN BROKER_USR_PASS

#define WIFI_CONN_SSID WIFI_SSID
#define WIFI_CONN_PASSWORD WIFI_PASS

#endif