#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <HTTPClient.h>


#define PIN_TRIG        5
#define PIN_ECHO        18
#define PIN_SPEED_POT   34
#define PIN_SERVO_BRAKE 13
#define PIN_BUZZER      12
#define PIN_LED_ALERT   2
#define PIN_LED_OK      15


#define CAN_ID_ADAS_FRONT    0x101  // Distance & Speed perception
#define CAN_ID_BRAKE_STATUS  0x050  // Actuated brake state

typedef struct {
    uint32_t id;        // 11-bit CAN arbitration identifier
    uint8_t  dlc;       // Data Length Code (0 - 8 bytes)
    uint8_t  data[8];   // Packed automotive signal bytes
    uint32_t timestamp; // System tick
} CAN_Frame_t;

// In-Memory Deterministic CAN Mailbox Queue
static QueueHandle_t xCanBusQueue = NULL;

// Transmit CAN frame onto virtual bus
bool CAN_Transmit(const char* sender_ecu, const CAN_Frame_t* frame) {
    if (xCanBusQueue == NULL) return false;
    BaseType_t res = xQueueSend(xCanBusQueue, frame, pdMS_TO_TICKS(10));
    if (res == pdTRUE) {
        Serial.printf("[CAN Tx] %-12s | ID: 0x%03X | DLC: %d | DATA: %02X %02X %02X %02X %02X %02X %02X %02X\n",
            sender_ecu, frame->id, frame->dlc,
            frame->data[0], frame->data[1], frame->data[2], frame->data[3],
            frame->data[4], frame->data[5], frame->data[6], frame->data[7]);
        return true;
    }
    return false;
}

// Receive CAN frame from virtual bus
bool CAN_Receive(CAN_Frame_t* frame, TickType_t timeout) {
    if (xCanBusQueue == NULL) return false;
    return xQueueReceive(xCanBusQueue, frame, timeout) == pdTRUE;
}


const char* WIFI_SSID       = "Wokwi-GUEST";
const char* WIFI_PASS       = "";
const char* MQTT_BROKER     = "broker.hivemq.com";
const int   MQTT_PORT       = 1883;
const char* MQTT_TOPIC_LIVE = "vehicle/telematics/live";
const char* MQTT_TOPIC_AEB  = "vehicle/safety/aeb_event";
const char* MQTT_TOPIC_CMD  = "vehicle/control/command";
const char* THINGSPEAK_KEY  = "VFIOURJO6GIWPHKZ";

typedef struct {
    float distance_cm;
    float speed_kmh;
    float ttc_sec;
    bool  emergency_brake;
} TelematicsData_t;

static TelematicsData_t g_telematics = {100.0f, 0.0f, 99.0f, false};
static portMUX_TYPE g_telematicsMux = portMUX_INITIALIZER_UNLOCKED;

Servo brakeServo;
WiFiClient espClient;
PubSubClient mqttClient(espClient);


void Task_ADAS_ECU(void *pvParameters) {
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);

    while (1) {
        // 1. Measure Ultrasonic Distance
        digitalWrite(PIN_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_TRIG, LOW);

        long duration = pulseIn(PIN_ECHO, HIGH, 30000); // 30ms timeout (~5m)
        float distance = (duration > 0) ? (duration * 0.0343f / 2.0f) : 400.0f;

        // 2. Read Vehicle Speed Simulator (Potentiometer: 0 - 140 km/h)
        int potVal = analogRead(PIN_SPEED_POT);
        float speed = (potVal / 4095.0f) * 140.0f;

        // 3. Serialize into CAN 2.0B Frame (0x101)
        CAN_Frame_t frame;
        frame.id  = CAN_ID_ADAS_FRONT;
        frame.dlc = 8;
        frame.timestamp = xTaskGetTickCount();

        uint16_t dist_raw  = (uint16_t)(distance * 10.0f); // 0.1 cm resolution
        uint16_t speed_raw = (uint16_t)(speed * 10.0f);    // 0.1 km/h resolution

        frame.data[0] = (dist_raw >> 8) & 0xFF;
        frame.data[1] = dist_raw & 0xFF;
        frame.data[2] = (speed_raw >> 8) & 0xFF;
        frame.data[3] = speed_raw & 0xFF;
        frame.data[4] = (distance < 25.0f) ? 1 : 0; // Hazard flag
        frame.data[5] = 0; frame.data[6] = 0; frame.data[7] = 0;

        // Broadcast onto simulated CAN bus
        CAN_Transmit("ADAS_ECU", &frame);

        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz automotive cycle
    }
}


void Task_SafetyGateway_ECU(void *pvParameters) {
    brakeServo.attach(PIN_SERVO_BRAKE);
    brakeServo.write(0); // 0 deg = Brake Released

    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED_ALERT, OUTPUT);
    pinMode(PIN_LED_OK, OUTPUT);

    CAN_Frame_t rxFrame;

    while (1) {
        if (CAN_Receive(&rxFrame, portMAX_DELAY)) {
            if (rxFrame.id == CAN_ID_ADAS_FRONT) {
                // Decode CAN signal bytes
                uint16_t dist_raw  = ((uint16_t)rxFrame.data[0] << 8) | rxFrame.data[1];
                uint16_t speed_raw = ((uint16_t)rxFrame.data[2] << 8) | rxFrame.data[3];

                float distance  = dist_raw / 10.0f;
                float speed_kmh = speed_raw / 10.0f;
                float speed_ms  = speed_kmh / 3.6f;

                // Calculate Time-To-Collision (TTC)
                float ttc = (speed_ms > 0.5f) ? (distance / 100.0f) / speed_ms : 99.0f;

                // Automated Emergency Braking (AEB) Logic
                bool aeb = (distance < 25.0f) || (ttc < 1.2f && speed_kmh > 15.0f);

                if (aeb) {
                    brakeServo.write(90);              // Clamp mechanical brake
                    digitalWrite(PIN_BUZZER, HIGH);    // Alarm buzzer
                    digitalWrite(PIN_LED_ALERT, HIGH); // Red warning LED
                    digitalWrite(PIN_LED_OK, LOW);
                } else {
                    brakeServo.write(0);               // Release brake
                    digitalWrite(PIN_BUZZER, LOW);
                    digitalWrite(PIN_LED_ALERT, LOW);
                    digitalWrite(PIN_LED_OK, HIGH);    // Green OK LED
                }

                // Broadcast Brake Status frame (0x050) back to CAN bus
                CAN_Frame_t brakeFrame;
                brakeFrame.id  = CAN_ID_BRAKE_STATUS;
                brakeFrame.dlc = 2;
                brakeFrame.data[0] = aeb ? 0x02 : 0x00; // 2 = Engaged, 0 = Released
                brakeFrame.data[1] = (uint8_t)min(99.0f, ttc);
                CAN_Transmit("GATEWAY_ECU", &brakeFrame);

                // Update shared telematics state safely
                portENTER_CRITICAL(&g_telematicsMux);
                g_telematics.distance_cm     = distance;
                g_telematics.speed_kmh       = speed_kmh;
                g_telematics.ttc_sec         = ttc;
                g_telematics.emergency_brake = aeb;
                portEXIT_CRITICAL(&g_telematicsMux);
            }
        }
    }
}


void onMqttCommand(char* topic, byte* payload, unsigned int length);

void Task_Telematics_Cloud(void *pvParameters) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(onMqttCommand);

    unsigned long lastThingSpeak = 0;

    while (1) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!mqttClient.connected()) {
                String clientId = "ESP32-TCU-" + String((uint32_t)ESP.getEfuseMac(), HEX);
                if (mqttClient.connect(clientId.c_str())) {
                    mqttClient.subscribe(MQTT_TOPIC_CMD);
                    Serial.println("[TCU Cloud] Connected to HiveMQ & subscribed to control commands");
                }
            } else {
                mqttClient.loop();
            }

            TelematicsData_t snap;
            portENTER_CRITICAL(&g_telematicsMux);
            snap = g_telematics;
            portEXIT_CRITICAL(&g_telematicsMux);

            // 1. MQTT Streaming (Every 2 seconds)
            char payload[160];
            snprintf(payload, sizeof(payload),
                "{\"speed\":%.1f,\"dist\":%.1f,\"ttc\":%.1f,\"brake\":%s}",
                snap.speed_kmh, snap.distance_cm, snap.ttc_sec,
                snap.emergency_brake ? "\"ACTIVE\"" : "\"RELEASED\""
            );
            mqttClient.publish(MQTT_TOPIC_LIVE, payload);

            if (snap.emergency_brake) {
                mqttClient.publish(MQTT_TOPIC_AEB, "{\"alert\":\"EMERGENCY_BRAKE_ACTUATED\"}");
            }

            // 2. ThingSpeak REST Update (Every 15 seconds)
            if (millis() - lastThingSpeak >= 15000) {
                lastThingSpeak = millis();
                if (strcmp(THINGSPEAK_KEY, "YOUR_THINGSPEAK_WRITE_API_KEY") != 0) {
                    HTTPClient http;
                    String url = "http://api.thingspeak.com/update?api_key=" + String(THINGSPEAK_KEY) +
                                 "&field1=" + String(snap.speed_kmh, 1) +
                                 "&field2=" + String(snap.distance_cm, 1) +
                                 "&field3=" + String(snap.emergency_brake ? 2 : 0) +
                                 "&field4=" + String(snap.ttc_sec, 1);
                    http.begin(url);
                    http.GET();
                    http.end();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Remote MQTT Command Callback
void onMqttCommand(char* topic, byte* payload, unsigned int length) {
    char msg[64];
    if (length >= sizeof(msg)) length = sizeof(msg) - 1;
    memcpy(msg, payload, length);
    msg[length] = '\0';

    Serial.printf("[TCU Cloud Rx] Command: %s\n", msg);
    if (strstr(msg, "REMOTE_BRAKE") != NULL) {
        brakeServo.write(90);
        digitalWrite(PIN_BUZZER, HIGH);
        digitalWrite(PIN_LED_ALERT, HIGH);
        digitalWrite(PIN_LED_OK, LOW);
        Serial.println("[GATEWAY] *** REMOTE OVERRIDE BRAKE ENGAGED ***");
    } else if (strstr(msg, "REMOTE_RELEASE") != NULL) {
        brakeServo.write(0);
        digitalWrite(PIN_BUZZER, LOW);
        digitalWrite(PIN_LED_ALERT, LOW);
        digitalWrite(PIN_LED_OK, HIGH);
        Serial.println("[GATEWAY] *** REMOTE OVERRIDE BRAKE RELEASED ***");
    }
}


void setup() {
    Serial.begin(115200);
    Serial.println("\n=======================================================");
    Serial.println("  DISTRIBUTED VEHICLE SAFETY & TELEMATICS (VIRTUAL CAN)");
    Serial.println("=======================================================");

    // Initialize Virtual CAN Bus Mailbox (Depth: 16 Frames)
    xCanBusQueue = xQueueCreate(16, sizeof(CAN_Frame_t));

    // Spawn Virtual ECUs as isolated FreeRTOS Tasks across dual cores
    xTaskCreatePinnedToCore(Task_ADAS_ECU,           "ADAS_ECU",    4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(Task_SafetyGateway_ECU,  "GATEWAY_ECU", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(Task_Telematics_Cloud,   "TCU_CLOUD",   4096, NULL, 1, NULL, 0);
}

void loop() {
    vTaskDelete(NULL);
}
