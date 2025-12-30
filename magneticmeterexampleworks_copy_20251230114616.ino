/*
  ESP8266 + LIS3MDL (SPI) Magnetic Water Meter Reader
  - MQTT publish: total_gallons (retained), flow_gpm (retained), rssi (retained), status (LWT + retained)
  - Home Assistant MQTT Discovery (retained config)
  - Long-term hardening:
      * availability (online/offline)
      * reconnect backoff (prevents heap fragmentation / reconnect storms)
      * EEPROM wear reduction + crash-safe persistence
      * sensor sanity bounds + hysteresis + time lockout
      * yields to keep ESP8266 WiFi stack happy
      * optional remote counter reset topic

  Wiring (NodeMCU ESP-12E):
    LIS3MDL VCC -> 3.3V
    LIS3MDL GND -> GND
    LIS3MDL SCL -> D5 (SCK / GPIO14)
    LIS3MDL SDA -> D7 (MOSI / GPIO13)
    LIS3MDL SDO -> D6 (MISO / GPIO12)
    LIS3MDL CS  -> D1 (GPIO5)  (do NOT use D8 or D0)

  Libraries:
    - Adafruit LIS3MDL
    - Adafruit Unified Sensor
    - PubSubClient
    - ArduinoJson (v6)
*/

#include <ESP8266WiFi.h>
#include <SPI.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

// ================= USER CONFIG =================

// WiFi
#define WIFI_SSID     "Hurricane"
#define WIFI_PASS     "crystals85"

// MQTT
#define MQTT_HOST     "192.168.1.158"
#define MQTT_PORT     1883
#define MQTT_USER     "mqtt"
#define MQTT_PASS     "doggy"

// Base topics
#define BASE_TOPIC    "home/water_meter"
#define HA_PREFIX     "homeassistant"

// Device identity (keep stable forever)
#define DEVICE_ID     "water_meter_esp8266"
#define DEVICE_NAME   "Water Meter"
#define MODEL_NAME    "ESP8266 + LIS3MDL"

// Meter calibration
#define GALLONS_PER_PULSE  1.0f

// Magnetic thresholds (tune these)
#define MAG_HIGH  120.0f
#define MAG_LOW    90.0f

// Pulse lockout to prevent double-counts due to noise/vibration (ms)
#define PULSE_LOCKOUT_MS  500UL

// Sensor poll interval (ms)
#define SENSOR_POLL_MS     50UL

// MQTT publish interval (ms)
#define PUBLISH_MS      10000UL

// EEPROM save interval (ms) to reduce wear
#define EEPROM_SAVE_MS  60000UL

// Sanity bounds for |Mag| to reject SPI glitches / NaNs
#define MAG_MIN_VALID    1.0f
#define MAG_MAX_VALID 1000.0f

// Optional remote reset command topic (publish "RESET")
#define ENABLE_REMOTE_RESET  1
#define RESET_TOPIC          BASE_TOPIC "/cmd"
#define RESET_PAYLOAD        "RESET"

// Troubleshooting to mqtt remotely
#define MAG_DEBUG_PUBLISH_MS 3000  // every 3 seconds


// =================================================

#define LIS3MDL_CS D1  // GPIO5 (boot-safe)

WiFiClient espClient;
PubSubClient mqtt(espClient);
Adafruit_LIS3MDL lis;

static unsigned long pulseCount = 0;
static unsigned long lastPulseMs = 0;
static bool triggered = false;
static unsigned long lastPulseAcceptedMs = 0;

// EEPROM layout
// [0..3] pulseCount (uint32_t)
// [4..7] signature
static const uint32_t EEPROM_SIG = 0xA55A1234;
static const int EEPROM_SIZE = 8;

static unsigned long lastEepromSaveMs = 0;
static unsigned long lastSensorPollMs = 0;
static unsigned long lastPublishMs = 0;
static unsigned long lastMqttAttemptMs = 0;

static bool discoveryPublished = false;

// ---------- Helpers ----------
static inline const char* cstr(const String& s) { return s.c_str(); }

void eepromLoad() {
  EEPROM.begin(EEPROM_SIZE);
  uint32_t storedCount = 0;
  uint32_t sig = 0;

  EEPROM.get(0, storedCount);
  EEPROM.get(4, sig);

  if (sig != EEPROM_SIG) {
    pulseCount = 0;
    EEPROM.put(0, (uint32_t)0);
    EEPROM.put(4, EEPROM_SIG);
    EEPROM.commit();
  } else {
    pulseCount = storedCount;
  }
}

void eepromSaveIfDue(bool force = false) {
  unsigned long now = millis();
  if (!force && (now - lastEepromSaveMs) < EEPROM_SAVE_MS) return;

  EEPROM.put(0, (uint32_t)pulseCount);
  EEPROM.put(4, EEPROM_SIG);
  EEPROM.commit();

  lastEepromSaveMs = now;
}

void wifiEnsureConnected() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP); // improves stability for always-on nodes
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(1500);
    yield();
    if (millis() - start > 20000UL) break; // don't hard-hang forever
  }
}

void mqttPublishRetained(const String& topic, const String& payload) {
  if (!mqtt.connected()) return;
  mqtt.publish(cstr(topic), cstr(payload), true);
}

void mqttPublishStatus(const char* status) {
  if (!mqtt.connected()) return;
  mqtt.publish(cstr(String(BASE_TOPIC) + "/status"), status, true);
}

String discoveryTopic(const String& suffix) {
  return String(HA_PREFIX) + "/sensor/" + String(DEVICE_ID) + "_" + suffix + "/config";
}

void publishDiscovery() {
  StaticJsonDocument<640> doc;
  char payload[640];
  size_t len;

  // Common device block
  auto addDeviceBlock = [&](StaticJsonDocument<640>& d) {
    JsonObject dev = d.createNestedObject("device");
    dev["identifiers"] = DEVICE_ID;
    dev["name"] = DEVICE_NAME;
    dev["manufacturer"] = "DIY";
    dev["model"] = MODEL_NAME;
  };

  // ---- MAG DEBUG SENSOR ----
  doc.clear();
  doc["name"] = "Water Meter Magnetic Field";
  doc["state_topic"] = String(BASE_TOPIC) + "/mag_debug";
  doc["payload_available"] = "online";
  doc["payload_not_available"] = "offline";
  doc["entity_category"] = "diagnostic";
  doc["unique_id"] = String(DEVICE_ID) + "_mag";
  doc["availability_topic"] = String(BASE_TOPIC) + "/status";
  addDeviceBlock(doc);
  len = serializeJson(doc, payload, sizeof(payload));
  mqtt.publish(cstr(discoveryTopic("mag")), (uint8_t*)payload, len, true);

  // ---- TOTAL GALLONS SENSOR ----
  doc.clear();
  doc["name"] = "Water Meter Total";
  doc["state_topic"] = String(BASE_TOPIC) + "/total_gallons";
  doc["unit_of_measurement"] = "gal";
  doc["device_class"] = "water";
  doc["state_class"] = "total_increasing";
  doc["payload_available"] = "online";
  doc["payload_not_available"] = "offline";
  doc["unique_id"] = String(DEVICE_ID) + "_total";
  doc["availability_topic"] = String(BASE_TOPIC) + "/status";
  addDeviceBlock(doc);
  len = serializeJson(doc, payload, sizeof(payload));
  mqtt.publish(cstr(discoveryTopic("total")), (uint8_t*)payload, len, true);

  // ---- FLOW RATE SENSOR ----
  doc.clear();
  doc["name"] = "Water Flow Rate";
  doc["state_topic"] = String(BASE_TOPIC) + "/flow_gpm";
  doc["unit_of_measurement"] = "gpm";
  doc["payload_available"] = "online";
  doc["payload_not_available"] = "offline";
  doc["unique_id"] = String(DEVICE_ID) + "_flow";
  doc["availability_topic"] = String(BASE_TOPIC) + "/status";
  addDeviceBlock(doc);
  len = serializeJson(doc, payload, sizeof(payload));
  mqtt.publish(cstr(discoveryTopic("flow")), (uint8_t*)payload, len, true);

  // ---- RSSI SENSOR ----
  doc.clear();
  doc["name"] = "Water Meter RSSI";
  doc["state_topic"] = String(BASE_TOPIC) + "/rssi";
  doc["unit_of_measurement"] = "dBm";
  doc["entity_category"] = "diagnostic";
  doc["payload_available"] = "online";
  doc["payload_not_available"] = "offline";
  doc["unique_id"] = String(DEVICE_ID) + "_rssi";
  doc["availability_topic"] = String(BASE_TOPIC) + "/status";
  addDeviceBlock(doc);
  len = serializeJson(doc, payload, sizeof(payload));
  mqtt.publish(cstr(discoveryTopic("rssi")), (uint8_t*)payload, len, true);
  discoveryPublished = true;
}



#if ENABLE_REMOTE_RESET
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Very small command surface area: only accept exact "RESET"
  if (strcmp(topic, RESET_TOPIC) != 0) return;

  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  if (msg == RESET_PAYLOAD) {
    pulseCount = 0;
    lastPulseMs = 0;
    triggered = false;
    lastPulseAcceptedMs = 0;
    eepromSaveIfDue(true);

    mqttPublishRetained(String(BASE_TOPIC) + "/total_gallons", "0.0");
    mqttPublishRetained(String(BASE_TOPIC) + "/flow_gpm", "0");
  }
}
#endif

bool mqttEnsureConnected() {
  if (mqtt.connected()) return true;

  unsigned long now = millis();
  if (now - lastMqttAttemptMs < 10000UL) return false; // backoff
  lastMqttAttemptMs = now;

  // Last Will: offline (retained)
  const String statusTopic = String(BASE_TOPIC) + "/status";
  bool ok = mqtt.connect(
    "water_meter_8266",
    MQTT_USER,
    MQTT_PASS,
    cstr(statusTopic),
    0,
    true,
    "offline"
  );

  if (ok) {
    mqttPublishStatus("online");
    if (!discoveryPublished) publishDiscovery();

#if ENABLE_REMOTE_RESET
    mqtt.subscribe(RESET_TOPIC);
#endif
  }

  return ok;
}

void publishState() {
  float totalGallons = (float)pulseCount * (float)GALLONS_PER_PULSE;

  mqttPublishRetained(String(BASE_TOPIC) + "/total_gallons", String(totalGallons, 1));

  // flow calculation: if last pulse recent, compute gpm
  if (lastPulseMs != 0 && (millis() - lastPulseMs) < 60000UL) {
    float gpm = 60000.0f / (float)(millis() - lastPulseMs);
    mqttPublishRetained(String(BASE_TOPIC) + "/flow_gpm", String(gpm, 2));
  } else {
    mqttPublishRetained(String(BASE_TOPIC) + "/flow_gpm", "0");
  }

  mqttPublishRetained(String(BASE_TOPIC) + "/rssi", String(WiFi.RSSI()));
}

bool initSensorSPI() {
  pinMode(LIS3MDL_CS, OUTPUT);
  digitalWrite(LIS3MDL_CS, HIGH); // deselect

  SPI.begin();
  delay(10);
  yield();

  if (!lis.begin_SPI(LIS3MDL_CS)) return false;

  lis.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis.setRange(LIS3MDL_RANGE_4_GAUSS);

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\nBooting water meter");

  // Sensor first (avoid SPI/WiFi contention during bring-up)
  if (!initSensorSPI()) {
    Serial.println("LIS3MDL not detected! Halting.");
    while (1) { delay(1000); yield(); }
  }
  Serial.println("LIS3MDL OK");

  eepromLoad();
  Serial.printf("pulseCount restored: %lu\n", pulseCount);

  wifiEnsureConnected();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
#if ENABLE_REMOTE_RESET
  mqtt.setCallback(mqttCallback);
#endif

  // Try connect early; loop() will keep retrying with backoff
  mqttEnsureConnected();
  publishState();
  lastPublishMs = millis();

  Serial.println("System ready");
}

void loop() {
  // Keep WiFi up
  wifiEnsureConnected();

  // Keep MQTT up (with backoff)
  mqttEnsureConnected();
  mqtt.loop();

  unsigned long now = millis();

  // ---- Sensor polling ----
  if (now - lastSensorPollMs >= SENSOR_POLL_MS) {
    lastSensorPollMs = now;

  sensors_event_t event;
  bool ok = lis.getEvent(&event);
  if (ok) {

    // 1) Calculate magnetic magnitude
    float mag = sqrtf(
      event.magnetic.x * event.magnetic.x +
      event.magnetic.y * event.magnetic.y +
      event.magnetic.z * event.magnetic.z
    );

    // ===== MQTT MAG DEBUG (REMOTE CALIBRATION) =====
static unsigned long lastMagDebugPub = 0;
if (millis() - lastMagDebugPub > MAG_DEBUG_PUBLISH_MS) {
  lastMagDebugPub = millis();

  mqttPublishRetained(
    String(BASE_TOPIC) + "/mag_debug",
    String(mag, 1)
  );
}
// =============================================


    // ===== TEMPORARY DEBUG OUTPUT =====
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 200) {   // 5 Hz update
      lastDebug = millis();
      Serial.printf(
        "X=%7.1f  Y=%7.1f  Z=%7.1f  |Mag|=%7.1f\n",
        event.magnetic.x,
        event.magnetic.y,
        event.magnetic.z,
        mag
      );
    }
    // =================================

      // sanity filter (reject glitches)
      if (isfinite(mag) && mag > MAG_MIN_VALID && mag < MAG_MAX_VALID) {
        // Hysteresis + time lockout
        if (!triggered && mag > MAG_HIGH && (now - lastPulseAcceptedMs) > PULSE_LOCKOUT_MS) {
          pulseCount++;
          lastPulseMs = now;
          lastPulseAcceptedMs = now;
          triggered = true;

          // Don't write EEPROM every pulse (wear); save on interval
          eepromSaveIfDue(false);

          Serial.printf("Pulse %lu |Mag|=%.2f\n", pulseCount, mag);
        }

        if (triggered && mag < MAG_LOW) {
          triggered = false;
        }
      }
    }
  }

  // ---- Periodic publish ----
  if (now - lastPublishMs >= PUBLISH_MS) {
    lastPublishMs = now;
    publishState();
    eepromSaveIfDue(false);
  }

  // Always yield on ESP8266 to avoid WDT resets
  yield();
}

/*
  Notes for HA discovery troubleshooting / cleanup:
  - If you previously published wrong discovery configs, clear retained configs once:
      Publish empty retained payloads to:
        homeassistant/sensor/water_meter_total/config
        homeassistant/sensor/water_meter_flow/config
        homeassistant/sensor/water_meter_rssi/config
    Then restart Home Assistant and reboot this ESP.

  Expected state topics:
    home/water_meter/total_gallons  (retained)
    home/water_meter/flow_gpm       (retained)
    home/water_meter/rssi           (retained)
    home/water_meter/status         (retained; online/offline)
    home/water_meter/cmd            (optional; publish "RESET")
*/
