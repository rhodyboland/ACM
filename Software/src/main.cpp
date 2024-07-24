#include <Arduino.h>
#include <PicoMQTT.h>

#include "acm.hpp"

#define CAR

#if __has_include("config.h")
#include "config.h"
#endif

#ifdef CAR
#ifndef WIFI_SSID
#define WIFI_SSID "ssid"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "password"
#endif
#endif

PicoMQTT::Server mqtt;

/*
Setup wifi and acm
*/
void setup() {
    // Setup serial
    Serial.begin(115200);

    // Connect to WiFi
    Serial.printf("Connecting to WiFi %s\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
    }
    Serial.printf("WiFi connected, IP: %s\n",
                  WiFi.localIP().toString().c_str());

    init_acm();

    // subscribe to a topic and attach a callback
    mqtt.subscribe("Remote/acm/outputs/#", [](char* topic, char* payload) {
        // payload might be binary, but PicoMQTT guarantees that it's
        // zero-terminated
        Serial.printf("Received message in topic '%s': %s\n", topic, payload);
        callback(topic, payload);
    });

    mqtt.begin();
}

/*
Main loop
*/
void loop() {
    mqtt.loop();
    sendData();
    readCS();
}