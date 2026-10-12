#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>


#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C

#define PIN_BTN_BRAKE   14
#define PIN_BTN_RELEASE 27
#define PIN_LED_ALERT   2

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


const char* WIFI_SSID       = "Wokwi-GUEST";
const char* WIFI_PASS       = "";
const char* MQTT_BROKER     = "broker.hivemq.com";
const int   MQTT_PORT       = 1883;
const char* MQTT_TOPIC_LIVE = "vehicle/telematics/live";
const char* MQTT_TOPIC_CMD  = "vehicle/control/command";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Vehicle Telemetry State
float g_speed = 0.0f;
float g_dist  = 0.0f;
float g_ttc   = 99.0f;
bool  g_brake = false;

void updateOLED() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // Header
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("FLEET MONITOR [TCU]");
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

    // Speed & Distance
    display.setCursor(0, 15);
    display.printf("SPD : %.1f km/h\n", g_speed);

    display.setCursor(0, 27);
    display.printf("DIST: %.1f cm\n", g_dist);

    display.setCursor(0, 39);
    display.printf("TTC : %.1f s\n", g_ttc);

    // Brake Status Banner
    display.setCursor(0, 52);
    if (g_brake) {
        display.println("BRAKE: >> ACTIVE <<");
        digitalWrite(PIN_LED_ALERT, HIGH);
    } else {
        display.println("BRAKE: RELEASED (OK)");
        digitalWrite(PIN_LED_ALERT, LOW);
    }

    display.display();
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    char jsonBuf[200];
    if (length >= sizeof(jsonBuf)) length = sizeof(jsonBuf) - 1;
    memcpy(jsonBuf, payload, length);
    jsonBuf[length] = '\0';

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, jsonBuf);
    if (!err) {
        g_speed = doc["speed"] | 0.0f;
        g_dist  = doc["dist"]  | 0.0f;
        g_ttc   = doc["ttc"]   | 99.0f;
        const char* b = doc["brake"] | "RELEASED";
        g_brake = (strcmp(b, "ACTIVE") == 0);

        updateOLED();
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(PIN_BTN_BRAKE,   INPUT_PULLUP);
    pinMode(PIN_BTN_RELEASE, INPUT_PULLUP);
    pinMode(PIN_LED_ALERT,   OUTPUT);

    // Initialize I2C OLED
    Wire.begin(21, 22);
    if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(10, 25);
        display.println("CONNECTING WIFI...");
        display.display();
    }

    // Connect WiFi & MQTT
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print(".");
    }
    Serial.println("\n[MONITOR] WiFi Connected!");

    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(onMqttMessage);

    String clientId = "FleetMon-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqttClient.connect(clientId.c_str())) {
        mqttClient.subscribe(MQTT_TOPIC_LIVE);
        Serial.println("[MONITOR] Subscribed to vehicle/telematics/live");
    }

    updateOLED();
}

void loop() {
    if (!mqttClient.connected()) {
        String clientId = "FleetMon-" + String((uint32_t)ESP.getEfuseMac(), HEX);
        if (mqttClient.connect(clientId.c_str())) {
            mqttClient.subscribe(MQTT_TOPIC_LIVE);
        }
    } else {
        mqttClient.loop();
    }

    // Check Button 1: Remote Emergency Brake (Active LOW)
    static bool lastBrakeBtn = HIGH;
    bool currBrakeBtn = digitalRead(PIN_BTN_BRAKE);
    if (lastBrakeBtn == HIGH && currBrakeBtn == LOW) {
        Serial.println("[MONITOR] Sending REMOTE_BRAKE command!");
        mqttClient.publish(MQTT_TOPIC_CMD, "REMOTE_BRAKE");
        delay(200); // Debounce
    }
    lastBrakeBtn = currBrakeBtn;

    // Check Button 2: Remote Brake Release (Active LOW)
    static bool lastRelBtn = HIGH;
    bool currRelBtn = digitalRead(PIN_BTN_RELEASE);
    if (lastRelBtn == HIGH && currRelBtn == LOW) {
        Serial.println("[MONITOR] Sending REMOTE_RELEASE command!");
        mqttClient.publish(MQTT_TOPIC_CMD, "REMOTE_RELEASE");
        delay(200); // Debounce
    }
    lastRelBtn = currRelBtn;

    delay(20);
}
