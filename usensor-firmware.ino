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

int on_sht = 0;
int on_motion = LOW;

float temperature_avg = 0.0f;
float temperature_now = 0.0f;
float temperature_lst = 0.0f;
float temperature_snd = 0.0f;

float humidity_avg = 0.0f;
float humidity_now = 0.0f;
float humidity_lst = 0.0f;
float humidity_snd = 0.0f;

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
  Serial.print("ip: ");
  Serial.println(WiFi.localIP());
  Serial.print("mac: ");
  Serial.println(WiFi.macAddress());
  
  mqtt.setServer("192.168.0.111", 1883);
  mqtt.setCallback(callback);
  
  on_sht = millis();
  on_motion = digitalRead(0);
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
        // Wait 2 seconds before retrying
        delay(2000);
      }
    }
  }
  
  if (millis() - on_sht >= 1000) {
    on_sht = millis();

    if (sht.isConnected()) {
      sht.read();

      temperature_now = sht.getTemperature();
      temperature_avg = (temperature_now + temperature_lst) / 2;
      temperature_avg = 0.5 * round(2.0 * temperature_avg);
      temperature_lst = temperature_now;
      
      humidity_now = sht.getHumidity();
      humidity_avg = (humidity_now + humidity_lst) / 2;
      humidity_avg = 0.5 * round(2.0 * humidity_avg);
      humidity_lst = humidity_now;
    }
  }
  
  if (temperature_snd != temperature_avg) {
    temperature_snd = temperature_avg;
    mqtt.publish("usensor/temperature/getvalue", String(temperature_snd).c_str());
    Serial.print("temperature: ");
    Serial.println(temperature_snd);
  }
  
  if (humidity_snd != humidity_avg) {
    humidity_snd = humidity_avg;
    mqtt.publish("usensor/humidity/getvalue", String(humidity_snd).c_str());
    Serial.print("humidity: ");
    Serial.println(humidity_snd);
  }
  
  if (digitalRead(0) == HIGH && on_motion == LOW) {
    on_motion = HIGH;
    mqtt.publish("usensor/motion/getvalue", "ON");
    Serial.println("montion: detected");
  } else if (digitalRead(0) == LOW && on_motion == HIGH) {
    on_motion = LOW;
    mqtt.publish("usensor/motion/getvalue", "OFF");
    Serial.println("montion: clear");
  }
  
  mqtt.loop();
}
