#ifndef DISPLAY
    #define DISPLAY 1
#endif

#ifndef SERIAL
    #define SERIAL 1
#endif

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SHT31.h>
#include <BH1750.h>

#ifdef DISPLAY
    #include <U8g2lib.h>
    #include <U8x8lib.h>

    U8X8_SSD1306_64X48_ER_HW_I2C display(U8X8_PIN_NONE);
#endif

#define LUX_FLUSH_CNT 60
#define LUX_SAMPLE_CNT 15
#define TEMP_FLUSH_CNT 60
#define TEMP_SAMPLE_CNT 15
#define HMDT_FLUSH_CNT 60
#define HMDT_SAMPLE_CNT 15
#define MOTION_PIN 0

char ssid[] = "";
char pass[] = "";

SHT31 sht;
BH1750 lightMeter;
WiFiClient wifi;
PubSubClient mqtt(wifi);

int on_sht = 0;
int on_motion = LOW;

int lux_cnt = 1;
float lux_avg = 0.0f;
float lux_snd = 0.0f;
float lux_tmp[LUX_SAMPLE_CNT] = { 0.0f };

int temperature_cnt = 1;
float temperature_avg = 0.0f;
float temperature_snd = 0.0f;
float temperature_tmp[TEMP_SAMPLE_CNT] = { 0.0f };

int humidity_cnt = 1;
float humidity_avg = 0.0f;
float humidity_snd = 0.0f;
float humidity_tmp[HMDT_SAMPLE_CNT] = { 0.0f };

void callback(char* t, byte* p, unsigned int l) {}

#ifdef DISPLAY
void display_measures() {
    display.clearDisplay();
    display.drawString(0, 0, "t:");
    display.drawString(2, 0, String(temperature_snd).c_str());
    display.drawString(0, 1, "h:");
    display.drawString(2, 1, String(humidity_snd).c_str());
    display.drawString(0, 2, "i:");
    display.drawString(2, 2, String(lux_snd).c_str());
    display.drawString(0, 3, "m:");
    display.drawString(2, 3, on_motion == HIGH ? "yes" : "no");
}
#endif

void setup() {
#ifdef SERIAL
    Serial.begin(9600);
#endif

    Wire.begin();
    Wire.setClock(100000);

    sht.begin(0x45);
    lightMeter.begin();

#ifdef DISPLAY
    display.begin();
    display.setPowerSave(0);
    display.setFont(u8x8_font_chroma48medium8_r);
    display.clearDisplay();
#endif

#ifdef SERIAL
    Serial.println();
    Serial.print("connecting to ");
    Serial.println(ssid);
#endif

#ifdef DISPLAY
    display.clearDisplay();
    display.drawString(0, 0, "connecting to:");
    display.drawString(0, 1, ssid);
#endif

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

    randomSeed(micros());

#ifdef SERIAL
    Serial.println("");
    Serial.println("wifi connected");
    Serial.print("ip: ");
    Serial.println(WiFi.localIP());
    Serial.print("mac: ");
    Serial.println(WiFi.macAddress());
#endif

#ifdef DISPLAY
    display.clearDisplay();
    display.drawString(0, 0, "wifi connected");
    display.drawString(0, 1, "ip:");
    display.drawString(0, 2, WiFi.localIP().toString().c_str());
    display.drawString(0, 3, "mac:");
    display.drawString(0, 4, WiFi.macAddress().c_str());
#endif

    mqtt.setServer("192.168.0.111", 1883);
    mqtt.setCallback(callback);

    pinMode(MOTION_PIN, INPUT_PULLUP);

    on_sht = millis();
    on_motion = digitalRead(MOTION_PIN);
}

void loop() {
    if (!mqtt.connected()) {
        // Loop until we're reconnected
        while (!mqtt.connected()) {
            // Attempt to connect
            if (mqtt.connect("usensor", "mosquitto", "password")) {
#ifdef SERIAL
                Serial.println("mqtt connected");
#endif

#ifdef DISPLAY
                display.clearDisplay();
                display.drawString(0, 0, "mqtt connected");
#endif

                // Once connected, publish an announcement...
                mqtt.publish("usensor/system/getonline", "online");
                mqtt.publish("usensor/motion/getvalue", on_motion == LOW ? "OFF" : "ON");
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

        lux_cnt++;
        humidity_cnt++;
        temperature_cnt++;
    
        // Lux SMA calculation
        for (int i = LUX_SAMPLE_CNT; i > 1; i--) {
            lux_tmp[i - 1] = lux_tmp[i - 2];
        }
        lux_tmp[0] = lightMeter.readLightLevel();

        lux_avg = 0.0f;
        for (int i = 0; i < LUX_SAMPLE_CNT; i++) {
            lux_avg += lux_tmp[i];
        }
        lux_avg /= LUX_SAMPLE_CNT;
        lux_avg = 0.5 * round(2.0 * lux_avg);

        if (sht.isConnected()) {
            sht.read();

            // Temperature SMA calculation
            for (int i = TEMP_SAMPLE_CNT; i > 1; i--) {
                temperature_tmp[i - 1] = temperature_tmp[i - 2];
            }
            temperature_tmp[0] = sht.getTemperature();

            temperature_avg = 0.0f;
            for (int i = 0; i < TEMP_SAMPLE_CNT; i++) {
                temperature_avg += temperature_tmp[i];
            }
            temperature_avg /= TEMP_SAMPLE_CNT;
            temperature_avg = 0.5 * round(2.0 * temperature_avg);
      
            // Humidity SMA calculation
            for (int i = HMDT_SAMPLE_CNT; i > 1; i--) {
                humidity_tmp[i - 1] = humidity_tmp[i - 2];
            }
            humidity_tmp[0] = sht.getHumidity();

            humidity_avg = 0.0f;
            for (int i = 0; i < HMDT_SAMPLE_CNT; i++) {
                humidity_avg += humidity_tmp[i];
            }
            humidity_avg /= HMDT_SAMPLE_CNT;
            humidity_avg = 0.5 * round(2.0 * humidity_avg);
        }
    }

    if (lux_snd != lux_avg || lux_cnt % LUX_FLUSH_CNT == 0) {
        lux_cnt = 1;
        lux_snd = lux_avg;

        mqtt.publish("usensor/illuminance/getvalue", String(lux_snd).c_str());

#ifdef SERIAL
        Serial.print("illuminance: ");
        Serial.println(lux_snd);
#endif

#ifdef DISPLAY
        display_measures();
#endif
    }
  
    if (temperature_snd != temperature_avg || temperature_cnt % TEMP_FLUSH_CNT == 0) {
        temperature_cnt = 1;
        temperature_snd = temperature_avg;

        mqtt.publish("usensor/temperature/getvalue", String(temperature_snd).c_str());

#ifdef SERIAL
        Serial.print("temperature: ");
        Serial.println(temperature_snd);
#endif

#ifdef DISPLAY
        display_measures();
#endif
    }
  
    if (humidity_snd != humidity_avg || humidity_cnt % HMDT_FLUSH_CNT == 0) {
        humidity_cnt = 1;
        humidity_snd = humidity_avg;

        mqtt.publish("usensor/humidity/getvalue", String(humidity_snd).c_str());

#ifdef SERIAL
        Serial.print("humidity: ");
        Serial.println(humidity_snd);
#endif

#ifdef DISPLAY
        display_measures();
#endif
    }
  
    if (digitalRead(0) == HIGH && on_motion == LOW) {
        on_motion = HIGH;
        mqtt.publish("usensor/motion/getvalue", "ON");

#ifdef SERIAL
        Serial.println("montion: detected");
#endif

#ifdef DISPLAY
        display_measures();
#endif
    } else if (digitalRead(0) == LOW && on_motion == HIGH) {
        on_motion = LOW;
        mqtt.publish("usensor/motion/getvalue", "OFF");

#ifdef SERIAL
        Serial.println("montion: clear");
#endif

#ifdef DISPLAY
        display_measures();
#endif
    }
  
    mqtt.loop();
}
