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

// Holds the current button state.
volatile int stateb;

// Holds the last time debounce was evaluated (in millis).
volatile long lastDebounceTime = 0;

// The delay threshold for debounce checking.
const int debounceDelay = 50;

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

// Gets called by the interrupt.
void onChange() {
  // Get the pin reading.
  int reading = digitalRead(BTN_PIN);

  // Ignore dupe readings.
  if(reading == stateb) return;

  boolean debounce = false;

  // Check to see if the change is within a debounce delay threshold.
  if((millis() - lastDebounceTime) <= debounceDelay) {
    debounce = true;
  }

  // This update to the last debounce check is necessary regardless of debounce state.
  lastDebounceTime = millis();

  // Ignore reads within a debounce delay threshold.
  if(debounce) return;

  // All is good, persist the reading as the state.
  stateb = reading;
  if(stateb) {
    state = !state;
    brightness = state ? 255 : 0;
    updated = true;
  }
}

void pirChange() {
  pirChanged = true;
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

  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT);
  attachInterrupt(BTN_PIN, onChange, CHANGE);
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
    mqttClient.subscribe(MQTT_COMMAND_TOPIC);
    register_device();
  }

  if (pirChanged) {
    StaticJsonBuffer<256> staticJsonBuffer;
    JsonObject& root = staticJsonBuffer.createObject();
    root["id"] = "kitchen";
    root["distance"] = 1;
    mqtt_publish(MQTT_PRESENCE_TOPIC, root);
    Serial.println("pir trigger");

//    state = 1;
//    brightness = 255;
//    updated = true;

    pirChanged = false;
  }

  mqttClient.loop();
  if(updated) {
    Serial.println("updating");
    updated = false;
    update();
  }
}
