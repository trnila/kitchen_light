#include <Arduino.h>
#include "config.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
char jsonBuffer[256];

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int brightness = 1023;
bool state = true;
void mqttcallback(const char *topic, byte* payload, unsigned int length) {
  Serial.println(topic);
  payload[length] = 0;
  //Serial.println(payload);

  StaticJsonBuffer<256> jsonBuffer_;
  JsonObject& root = jsonBuffer_.parseObject(payload);


  if(root.containsKey("brightness")) {
    brightness = root["brightness"];
  }

  if(root.containsKey("state")) {
    state = root["state"] == "ON";
  }


  if(state) {
    analogWrite(D7, map(brightness, 0, 255, 0, 1023));
  } else {
    analogWrite(D7, 0);
  }

  StaticJsonBuffer<256> res;
  JsonObject& rroot = res.createObject();
  rroot["state"] = state ? "ON" : "OFF";
  rroot["brightness"] = brightness;
  rroot.printTo(jsonBuffer, sizeof(jsonBuffer));
  Serial.println(jsonBuffer);
  if(!mqttClient.publish(MQTT_STATE_TOPIC, jsonBuffer, true)) {
    Serial.println("publish failed");
    Serial.println(mqttClient.state());
  }
}

void bgn() {
  StaticJsonBuffer<256> staticJsonBuffer;
  JsonObject& root = staticJsonBuffer.createObject();
  root["name"] = "BedLight";
  root["platform"] = "mqtt_json";
  root["state_topic"] = MQTT_STATE_TOPIC;
  root["command_topic"] = MQTT_COMMAND_TOPIC;
  root["brightness"] = true;
  root.printTo(jsonBuffer, sizeof(jsonBuffer));
  if(!mqttClient.publish(MQTT_CONFIG_TOPIC, jsonBuffer)) {
    Serial.println("publish failed");
    Serial.println(mqttClient.state());
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nConnected, IP address: ");
  Serial.println(WiFi.localIP());

  mqttClient.setServer(CONFIG_MQTT_ADDR, CONFIG_MQTT_PORT);
  mqttClient.setCallback(mqttcallback);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(D7, OUTPUT);
}

void loop() {
  if(!mqttClient.connected()) {
    Serial.println("reconnecting");
    if(!mqttClient.connect("bedlight", MQTT_USERNAME, MQTT_PASSWORD, MQTT_STATUS_TOPIC, 0, 1, "dead")) {
      Serial.println("mqtt failed");
    }
    mqttClient.subscribe(MQTT_COMMAND_TOPIC);
    bgn();
  }

  mqttClient.loop();

  /*for(int i = 0; i < PWMRANGE; i++) {
    analogWrite(D7, i);
    delay(10);
  }*/
  delay(3000);
}
