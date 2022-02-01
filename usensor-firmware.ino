#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "Wire.h"
#include "SHT31.h"

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "";
char pass[] = "";

SHT31 sht;
WiFiClient wifi;
PubSubClient mqtt(wifi);

int on_time = 0;
int on_motion = 0;

void callback(char* t, byte* p, unsigned int l) {}

void setup() {
  Serial.begin(9600);
  
  Wire.begin();
  sht.begin(0x45);
  Wire.setClock(100000);

  uint16_t stat = sht.readStatus();
  Serial.print(stat, HEX);
  Serial.println();

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("wifi connected");
  Serial.println("ip address: ");
  Serial.println(WiFi.localIP());
  
  mqtt.setServer("192.168.0.111", 1883);
  mqtt.setCallback(callback);
  
  on_time = millis();
  on_motion = digitalRead(2);
}

void loop() {
  if (!mqtt.connected()) {
    // Loop until we're reconnected
    while (!mqtt.connected()) {
      // Attempt to connect
      if (mqtt.connect("usensor", "mosquitto", "password")) {
        Serial.println("mqtt connected");
        // Once connected, publish an announcement...
        mqtt.publish("usensor/system/getonline", "online");
      } else {
        Serial.print("mqtt connection failed, rc=");
        Serial.print(mqtt.state());
        Serial.println(" try again in 2 seconds");
        // Wait 5 seconds before retrying
        delay(2000);
      }
    }
  }

  mqtt.loop();
  
  if (millis() - on_time >= 5000) {
    on_time = millis();

    sht.read();
    mqtt.publish("usensor/temperature/getvalue", String(sht.getTemperature()).c_str());
    mqtt.publish("usensor/humidity/getvalue", String(sht.getHumidity()).c_str());
  }
  
  if (digitalRead(2) && on_motion == 0) {
    on_motion = 1;
    mqtt.publish("usensor/motion/getvalue", String(on_motion).c_str());
  } else if (on_motion) {
    on_motion = 0;
    mqtt.publish("usensor/motion/getvalue", String(on_motion).c_str());
  }
}
