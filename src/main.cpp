#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
char jsonBuffer[256];
bool updated = false;
bool pirChanged = false;

int brightness = 255;
bool state = true;

unsigned int lastTime;

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void mqtt_publish(const char* topic, const JsonObject &object) {
  object.printTo(jsonBuffer, sizeof(jsonBuffer));
  Serial.print("SEND: ");
  Serial.println(jsonBuffer);
  if(!mqttClient.publish(topic, jsonBuffer, true)) {
    Serial.println("publish failed");
    Serial.println(mqttClient.state());
  }
}

void update() {
  if(state) {
    analogWrite(LED_PIN, map(brightness, 0, 255, 0, 1023));
  } else {
    analogWrite(LED_PIN, 0);
  }

  StaticJsonBuffer<256> res;
  JsonObject& rroot = res.createObject();
  rroot["state"] = state ? "ON" : "OFF";
  rroot["brightness"] = brightness;
  mqtt_publish(MQTT_STATE_TOPIC, rroot);
}


void mqttcallback(const char *topic, byte* payload, unsigned int length) {
  Serial.println(topic);
  payload[length] = 0;

  StaticJsonBuffer<256> jsonBuffer_;
  JsonObject& root = jsonBuffer_.parseObject(payload);


  if(root.containsKey("brightness")) {
    brightness = root["brightness"];
    Serial.println("br: " + brightness);
  }

  if(root.containsKey("state")) {
    state = root["state"] == "ON";
    Serial.println("state: " + state);
  }
  updated = true;
}

void register_device() {
  StaticJsonBuffer<256> staticJsonBuffer;
  JsonObject& root = staticJsonBuffer.createObject();
  root["name"] = DEVICE_NAME;
  root["platform"] = "mqtt_json";
  root["state_topic"] = MQTT_STATE_TOPIC;
  root["command_topic"] = MQTT_COMMAND_TOPIC;
  root["brightness"] = true;
  mqtt_publish(MQTT_CONFIG_TOPIC, root);
}

void pirChange() {
  pirChanged = true;
  Serial.println("p");
}

void reset() {
  digitalWrite(LED_PIN, LOW);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  reset();

  Serial.begin(115200);

  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nConnected, IP address: ");
  Serial.println(WiFi.localIP());

  mqttClient.setServer(CONFIG_MQTT_ADDR, CONFIG_MQTT_PORT);
  mqttClient.setCallback(mqttcallback);

  attachInterrupt(PIR_PIN, pirChange, RISING);
  Serial.println("Initialized");
}

void loop() {
  if(!mqttClient.connected()) {
    Serial.println("reconnecting");
    if(!mqttClient.connect(DEVICE_NAME, MQTT_USERNAME, MQTT_PASSWORD, MQTT_STATUS_TOPIC, 0, 1, "dead")) {
      Serial.println("mqtt failed");
      delay(1000);
      return;
    }
    reset();
    mqttClient.subscribe(MQTT_COMMAND_TOPIC);
    register_device();
  }

  if(millis() - lastTime > 5000) {
    lastTime = millis();

    if(digitalRead(PIR_PIN)) {
      pirChanged = true;
    }
  }

  if (pirChanged) {
    StaticJsonBuffer<256> staticJsonBuffer;
    JsonObject& root = staticJsonBuffer.createObject();
    root["id"] = "kitchen";
    root["distance"] = 1;
    mqtt_publish(MQTT_PRESENCE_TOPIC, root);
    Serial.println("pir trigger");

    pirChanged = false;
  }

  mqttClient.loop();
  if(updated) {
    Serial.println("updating");
    updated = false;
    update();
  }
}
