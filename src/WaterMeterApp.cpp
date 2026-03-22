#include "WaterMeterApp.h"

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <Update.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <string.h>

using DeviceWebServer = WebServer;
static const char PLATFORM_NAME[] = "ESP32";

#ifndef LIS3MDL_CS_PIN
#define LIS3MDL_CS_PIN 5
#endif

static const char DEFAULT_WIFI_SSID[] = "your-wifi-ssid";
static const char DEFAULT_WIFI_PASS[] = "";
static const char DEFAULT_MQTT_HOST[] = "mqtt.local";
static const uint16_t DEFAULT_MQTT_PORT = 1883;
static const char DEFAULT_MQTT_USER[] = "";
static const char DEFAULT_MQTT_PASS[] = "";
static const char DEFAULT_HA_PREFIX[] = "homeassistant";
static const char DEFAULT_BASE_TOPIC[] = "home/water_meter";
static const char DEFAULT_DEVICE_ID[] = "water_meter_esp32";
static const char DEFAULT_DEVICE_NAME[] = "Water Meter";
static const char DEFAULT_MODEL_NAME[] = "ESP32 + LIS3MDL";

static const float DEFAULT_GALLONS_PER_PULSE = 0.0225f;
static const float DEFAULT_MAG_HIGH = 29.0f;
static const float DEFAULT_MAG_LOW = 25.0f;
static const uint32_t DEFAULT_PULSE_LOCKOUT_MS = 250UL;
static const uint32_t DEFAULT_SENSOR_POLL_MS = 10UL;
static const uint32_t DEFAULT_PUBLISH_MS = 10000UL;
static const uint32_t DEFAULT_FAST_PUBLISH_MS = 3000UL;
static const float DEFAULT_FLOW_FAST_THRESHOLD = 0.5f;
static const uint32_t DEFAULT_PERSIST_SAVE_MS = 60000UL;
static const float DEFAULT_MAG_MIN_VALID = 1.0f;
static const float DEFAULT_MAG_MAX_VALID = 400.0f;
static const uint8_t DEFAULT_ENABLE_REMOTE_RESET = 1;
static const float DEFAULT_LEAK_MIN_GPM = 0.05f;
static const uint32_t DEFAULT_LEAK_MIN_DURATION_MS = 10UL * 60UL * 1000UL;
static const uint32_t DEFAULT_MAG_DEBUG_PUBLISH_MS = 3000UL;

static const uint32_t STORAGE_MAGIC = 0x574D5354;
static const uint16_t STORAGE_VERSION = 1;
static const size_t STORAGE_BYTES = 1024;
static const size_t MQTT_BUFFER_BYTES = 768;
static const uint32_t WIFI_RETRY_MS = 10000UL;
static const uint32_t MQTT_RETRY_MS = 10000UL;
static const uint32_t RESTART_DELAY_MS = 1500UL;

struct AppSettings {
  char wifiSsid[33];
  char wifiPass[65];
  char mqttHost[65];
  uint16_t mqttPort;
  char mqttUser[33];
  char mqttPass[33];
  char baseTopic[65];
  char haPrefix[33];
  char deviceId[33];
  char deviceName[33];
  char modelName[33];
  float gallonsPerPulse;
  float magHigh;
  float magLow;
  uint32_t pulseLockoutMs;
  uint32_t sensorPollMs;
  uint32_t publishMs;
  uint32_t fastPublishMs;
  float flowFastThreshold;
  uint32_t persistSaveMs;
  float magMinValid;
  float magMaxValid;
  uint8_t enableRemoteReset;
  float leakMinGpm;
  uint32_t leakMinDurationMs;
  uint32_t magDebugPublishMs;
};

struct PersistedState {
  uint32_t magic;
  uint16_t version;
  uint16_t length;
  uint32_t pulseCount;
  AppSettings settings;
};

static_assert(sizeof(PersistedState) <= STORAGE_BYTES, "Persisted state exceeds storage allocation");

static WiFiClient espClient;
static PubSubClient mqtt(espClient);
static Adafruit_LIS3MDL lis;
static DeviceWebServer webServer(80);

static AppSettings settings;
static unsigned long pulseCount = 0;
static unsigned long lastPulseMs = 0;
static bool triggered = false;
static unsigned long lastPulseAcceptedMs = 0;
static unsigned long lastPersistenceSaveMs = 0;
static unsigned long lastSensorPollMs = 0;
static unsigned long lastPublishMs = 0;
static unsigned long lastWifiAttemptMs = 0;
static unsigned long lastMqttAttemptMs = 0;
static unsigned long lastFlowCalcMs = 0;
static unsigned long pulseCountAtLastFlowCalc = 0;
static unsigned long leakStartMs = 0;
static unsigned long lastMagDebugPublishMs = 0;
static unsigned long restartScheduledMs = 0;
static float flowGpmSmoothed = 0.0f;
static bool discoveryPublished = false;
static bool leakActive = false;
static bool leakStatePublished = false;
static bool wifiWasConnected = false;
static bool restartPending = false;
static bool sensorAvailable = false;
static bool otaUpdateSuccess = false;
static bool otaUpdateStarted = false;
static String otaUpdateError;

static inline const char* cstr(const String& value) { return value.c_str(); }

template <size_t N>
static void copyText(char (&dest)[N], const char* src) {
  if (src == nullptr) src = "";
  strncpy(dest, src, N - 1);
  dest[N - 1] = '\0';
}

template <size_t N>
static void copyText(char (&dest)[N], const String& src) {
  copyText(dest, src.c_str());
}

template <size_t N>
static void ensureNonEmpty(char (&dest)[N], const char* fallback) {
  dest[N - 1] = '\0';
  if (dest[0] == '\0') copyText(dest, fallback);
}

template <typename T>
static T clampValue(T value, T minimumValue, T maximumValue) {
  if (value < minimumValue) return minimumValue;
  if (value > maximumValue) return maximumValue;
  return value;
}

template <size_t N>
static void trimBuffer(char (&value)[N]) {
  String trimmed = value;
  trimmed.trim();
  copyText(value, trimmed);
}

template <size_t N>
static void trimTopicBuffer(char (&value)[N], const char* fallback) {
  trimBuffer(value);
  size_t length = strlen(value);
  while (length > 0 && value[length - 1] == '/') {
    value[length - 1] = '\0';
    length--;
  }
  ensureNonEmpty(value, fallback);
}

static void loadDefaultSettings(AppSettings& target) {
  memset(&target, 0, sizeof(target));

  copyText(target.wifiSsid, DEFAULT_WIFI_SSID);
  copyText(target.wifiPass, DEFAULT_WIFI_PASS);
  copyText(target.mqttHost, DEFAULT_MQTT_HOST);
  target.mqttPort = DEFAULT_MQTT_PORT;
  copyText(target.mqttUser, DEFAULT_MQTT_USER);
  copyText(target.mqttPass, DEFAULT_MQTT_PASS);
  copyText(target.baseTopic, DEFAULT_BASE_TOPIC);
  copyText(target.haPrefix, DEFAULT_HA_PREFIX);
  copyText(target.deviceId, DEFAULT_DEVICE_ID);
  copyText(target.deviceName, DEFAULT_DEVICE_NAME);
  copyText(target.modelName, DEFAULT_MODEL_NAME);
  target.gallonsPerPulse = DEFAULT_GALLONS_PER_PULSE;
  target.magHigh = DEFAULT_MAG_HIGH;
  target.magLow = DEFAULT_MAG_LOW;
  target.pulseLockoutMs = DEFAULT_PULSE_LOCKOUT_MS;
  target.sensorPollMs = DEFAULT_SENSOR_POLL_MS;
  target.publishMs = DEFAULT_PUBLISH_MS;
  target.fastPublishMs = DEFAULT_FAST_PUBLISH_MS;
  target.flowFastThreshold = DEFAULT_FLOW_FAST_THRESHOLD;
  target.persistSaveMs = DEFAULT_PERSIST_SAVE_MS;
  target.magMinValid = DEFAULT_MAG_MIN_VALID;
  target.magMaxValid = DEFAULT_MAG_MAX_VALID;
  target.enableRemoteReset = DEFAULT_ENABLE_REMOTE_RESET;
  target.leakMinGpm = DEFAULT_LEAK_MIN_GPM;
  target.leakMinDurationMs = DEFAULT_LEAK_MIN_DURATION_MS;
  target.magDebugPublishMs = DEFAULT_MAG_DEBUG_PUBLISH_MS;
}

static void sanitizeSettings(AppSettings& target) {
  trimBuffer(target.wifiSsid);
  trimBuffer(target.mqttHost);
  trimBuffer(target.mqttUser);
  trimBuffer(target.deviceId);
  trimBuffer(target.deviceName);
  trimBuffer(target.modelName);
  trimTopicBuffer(target.baseTopic, DEFAULT_BASE_TOPIC);
  trimTopicBuffer(target.haPrefix, DEFAULT_HA_PREFIX);

  ensureNonEmpty(target.wifiSsid, DEFAULT_WIFI_SSID);
  ensureNonEmpty(target.mqttHost, DEFAULT_MQTT_HOST);
  ensureNonEmpty(target.deviceId, DEFAULT_DEVICE_ID);
  ensureNonEmpty(target.deviceName, DEFAULT_DEVICE_NAME);
  ensureNonEmpty(target.modelName, DEFAULT_MODEL_NAME);

  target.mqttPort = clampValue<uint16_t>(target.mqttPort, 1, 65535);

  if (!isfinite(target.gallonsPerPulse) || target.gallonsPerPulse <= 0.0f) target.gallonsPerPulse = DEFAULT_GALLONS_PER_PULSE;
  if (!isfinite(target.magHigh)) target.magHigh = DEFAULT_MAG_HIGH;
  if (!isfinite(target.magLow)) target.magLow = DEFAULT_MAG_LOW;
  if (!isfinite(target.flowFastThreshold) || target.flowFastThreshold < 0.0f) target.flowFastThreshold = DEFAULT_FLOW_FAST_THRESHOLD;
  if (!isfinite(target.magMinValid) || target.magMinValid <= 0.0f) target.magMinValid = DEFAULT_MAG_MIN_VALID;
  if (!isfinite(target.magMaxValid) || target.magMaxValid <= target.magMinValid) target.magMaxValid = DEFAULT_MAG_MAX_VALID;
  if (!isfinite(target.leakMinGpm) || target.leakMinGpm < 0.0f) target.leakMinGpm = DEFAULT_LEAK_MIN_GPM;

  target.pulseLockoutMs = clampValue<uint32_t>(target.pulseLockoutMs, 20UL, 10000UL);
  target.sensorPollMs = clampValue<uint32_t>(target.sensorPollMs, 5UL, 500UL);
  target.publishMs = clampValue<uint32_t>(target.publishMs, 1000UL, 300000UL);
  target.fastPublishMs = clampValue<uint32_t>(target.fastPublishMs, 1000UL, target.publishMs);
  target.persistSaveMs = clampValue<uint32_t>(target.persistSaveMs, 5000UL, 3600000UL);
  target.leakMinDurationMs = clampValue<uint32_t>(target.leakMinDurationMs, 1000UL, 24UL * 60UL * 60UL * 1000UL);
  target.magDebugPublishMs = clampValue<uint32_t>(target.magDebugPublishMs, 0UL, 3600000UL);
  target.enableRemoteReset = target.enableRemoteReset ? 1 : 0;

  if (target.magHigh <= target.magLow) {
    target.magHigh = target.magLow + 1.0f;
  }
}

static String baseTopic() { return String(settings.baseTopic); }
static String statusTopic() { return baseTopic() + "/status"; }
static String totalGallonsTopic() { return baseTopic() + "/total_gallons"; }
static String flowTopic() { return baseTopic() + "/flow_gpm"; }
static String rssiTopic() { return baseTopic() + "/rssi"; }
static String leakTopic() { return baseTopic() + "/leak"; }
static String magDebugTopic() { return baseTopic() + "/mag_debug"; }
static String resetCommandTopic() { return baseTopic() + "/cmd"; }
static String discoveryTopic(const char* component, const char* suffix) {
  return String(settings.haPrefix) + "/" + component + "/" + String(settings.deviceId) + "_" + suffix + "/config";
}

static String htmlEscape(const String& input) {
  String output;
  output.reserve(input.length() + 16);

  for (size_t i = 0; i < input.length(); i++) {
    char ch = input[i];
    switch (ch) {
      case '&': output += F("&amp;"); break;
      case '<': output += F("&lt;"); break;
      case '>': output += F("&gt;"); break;
      case '"': output += F("&quot;"); break;
      case '\'': output += F("&#39;"); break;
      default: output += ch; break;
    }
  }

  return output;
}

static String htmlEscape(const char* input) {
  return htmlEscape(String(input ? input : ""));
}

static float totalGallonsValue() {
  return (float)pulseCount * settings.gallonsPerPulse;
}

static String firmwareUpdateErrorString() {
  return String("Update error code ") + String(Update.getError());
}

static bool beginFirmwareUpdate() {
  return Update.begin(UPDATE_SIZE_UNKNOWN);
}

static void cancelFirmwareUpdate() {
  Update.abort();
}

static void saveState(bool force = false) {
  unsigned long now = millis();
  if (!force && (now - lastPersistenceSaveMs) < settings.persistSaveMs) return;

  PersistedState persisted = {};
  persisted.magic = STORAGE_MAGIC;
  persisted.version = STORAGE_VERSION;
  persisted.length = sizeof(PersistedState);
  persisted.pulseCount = pulseCount;
  persisted.settings = settings;

  EEPROM.put(0, persisted);
  EEPROM.commit();
  lastPersistenceSaveMs = now;
}

static void loadState() {
  EEPROM.begin(STORAGE_BYTES);

  PersistedState persisted = {};
  EEPROM.get(0, persisted);

  if (persisted.magic == STORAGE_MAGIC &&
      persisted.version == STORAGE_VERSION &&
      persisted.length == sizeof(PersistedState)) {
    settings = persisted.settings;
    pulseCount = persisted.pulseCount;
    sanitizeSettings(settings);
  } else {
    loadDefaultSettings(settings);
    sanitizeSettings(settings);
    pulseCount = 0;
    saveState(true);
  }
}

static void applyWifiClientSettings() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(settings.deviceId);

  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
}

static bool wifiEnsureConnected() {
  if (WiFi.status() == WL_CONNECTED) return true;

  unsigned long now = millis();
  if ((now - lastWifiAttemptMs) < WIFI_RETRY_MS) return false;

  applyWifiClientSettings();
  WiFi.disconnect(false);
  WiFi.begin(settings.wifiSsid, settings.wifiPass);
  lastWifiAttemptMs = now;

  Serial.printf("WiFi connect attempt to %s\n", settings.wifiSsid);
  return false;
}

static void logWifiTransitions() {
  bool connected = (WiFi.status() == WL_CONNECTED);
  if (connected == wifiWasConnected) return;

  wifiWasConnected = connected;
  if (connected) {
    Serial.print("WiFi connected. IP=");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi disconnected");
  }
}

static void mqttPublishRetained(const String& topic, const String& payload) {
  if (!mqtt.connected()) return;
  mqtt.publish(cstr(topic), cstr(payload), true);
}

static void mqttPublishStatus(const char* status) {
  if (!mqtt.connected()) return;
  String topic = statusTopic();
  mqtt.publish(topic.c_str(), status, true);
}

static void mqttPublishLeak(const char* state) {
  if (!mqtt.connected()) return;
  String topic = leakTopic();
  mqtt.publish(topic.c_str(), state, true);
}

static void publishDiscovery() {
  StaticJsonDocument<768> doc;
  char payload[768];
  size_t length = 0;

  auto addDeviceBlock = [&](StaticJsonDocument<768>& target) {
    JsonObject device = target.createNestedObject("device");
    device["identifiers"] = settings.deviceId;
    device["name"] = settings.deviceName;
    device["manufacturer"] = "DIY";
    device["model"] = settings.modelName;
  };

  doc.clear();
  doc["name"] = "Water Meter Magnetic Field";
  doc["state_topic"] = magDebugTopic();
  doc["payload_available"] = "online";
  doc["payload_not_available"] = "offline";
  doc["entity_category"] = "diagnostic";
  doc["unique_id"] = String(settings.deviceId) + "_mag";
  doc["availability_topic"] = statusTopic();
  addDeviceBlock(doc);
  length = serializeJson(doc, payload, sizeof(payload));
  mqtt.publish(discoveryTopic("sensor", "mag").c_str(), (uint8_t*)payload, length, true);

  doc.clear();
  doc["name"] = "Water Meter Total";
  doc["state_topic"] = totalGallonsTopic();
  doc["unit_of_measurement"] = "gal";
  doc["device_class"] = "water";
  doc["state_class"] = "total_increasing";
  doc["payload_available"] = "online";
  doc["payload_not_available"] = "offline";
  doc["unique_id"] = String(settings.deviceId) + "_total";
  doc["availability_topic"] = statusTopic();
  addDeviceBlock(doc);
  length = serializeJson(doc, payload, sizeof(payload));
  mqtt.publish(discoveryTopic("sensor", "total").c_str(), (uint8_t*)payload, length, true);

  doc.clear();
  doc["name"] = "Water Flow Rate";
  doc["state_topic"] = flowTopic();
  doc["unit_of_measurement"] = "gpm";
  doc["payload_available"] = "online";
  doc["payload_not_available"] = "offline";
  doc["unique_id"] = String(settings.deviceId) + "_flow";
  doc["availability_topic"] = statusTopic();
  addDeviceBlock(doc);
  length = serializeJson(doc, payload, sizeof(payload));
  mqtt.publish(discoveryTopic("sensor", "flow").c_str(), (uint8_t*)payload, length, true);

  doc.clear();
  doc["name"] = "Water Meter RSSI";
  doc["state_topic"] = rssiTopic();
  doc["unit_of_measurement"] = "dBm";
  doc["entity_category"] = "diagnostic";
  doc["payload_available"] = "online";
  doc["payload_not_available"] = "offline";
  doc["unique_id"] = String(settings.deviceId) + "_rssi";
  doc["availability_topic"] = statusTopic();
  addDeviceBlock(doc);
  length = serializeJson(doc, payload, sizeof(payload));
  mqtt.publish(discoveryTopic("sensor", "rssi").c_str(), (uint8_t*)payload, length, true);

  doc.clear();
  doc["name"] = "Water Leak (flow)";
  doc["state_topic"] = leakTopic();
  doc["payload_on"] = "ON";
  doc["payload_off"] = "OFF";
  doc["device_class"] = "problem";
  doc["entity_category"] = "diagnostic";
  doc["payload_available"] = "online";
  doc["payload_not_available"] = "offline";
  doc["unique_id"] = String(settings.deviceId) + "_leak";
  doc["availability_topic"] = statusTopic();
  addDeviceBlock(doc);
  length = serializeJson(doc, payload, sizeof(payload));
  mqtt.publish(discoveryTopic("binary_sensor", "leak").c_str(), (uint8_t*)payload, length, true);

  discoveryPublished = true;
}

static void resetMeterCount() {
  pulseCount = 0;
  lastPulseMs = 0;
  lastPulseAcceptedMs = 0;
  triggered = false;
  lastFlowCalcMs = 0;
  pulseCountAtLastFlowCalc = 0;
  flowGpmSmoothed = 0.0f;
  leakStartMs = 0;
  leakActive = false;
  leakStatePublished = false;
  saveState(true);
}

static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (!settings.enableRemoteReset) return;

  String expectedTopic = resetCommandTopic();
  if (expectedTopic != topic) return;

  String message;
  message.reserve(length);
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
  message.trim();

  if (message == "RESET") {
    resetMeterCount();
    mqttPublishRetained(totalGallonsTopic(), "0.0");
    mqttPublishRetained(flowTopic(), "0");
    mqttPublishLeak("OFF");
  }
}

static bool mqttEnsureConnected() {
  if (mqtt.connected()) return true;
  if (WiFi.status() != WL_CONNECTED) return false;

  unsigned long now = millis();
  if ((now - lastMqttAttemptMs) < MQTT_RETRY_MS) return false;
  lastMqttAttemptMs = now;

  mqtt.setServer(settings.mqttHost, settings.mqttPort);

  String clientId = settings.deviceId;
  String willTopic = statusTopic();
  bool connected = mqtt.connect(
    clientId.c_str(),
    settings.mqttUser,
    settings.mqttPass,
    willTopic.c_str(),
    0,
    true,
    "offline"
  );

  if (connected) {
    mqttPublishStatus("online");
    if (!discoveryPublished) publishDiscovery();
    mqttPublishLeak(leakActive ? "ON" : "OFF");
    leakStatePublished = true;

    if (settings.enableRemoteReset) {
      String topic = resetCommandTopic();
      mqtt.subscribe(topic.c_str());
    }

    Serial.println("MQTT connected");
  }

  return connected;
}

static void updateLeakStatus(float gpm, unsigned long now) {
  if (gpm >= settings.leakMinGpm) {
    if (leakStartMs == 0) leakStartMs = now;
    if (!leakActive && (now - leakStartMs >= settings.leakMinDurationMs)) {
      leakActive = true;
      mqttPublishLeak("ON");
      leakStatePublished = true;
    }
  } else {
    leakStartMs = 0;
    if (leakActive || !leakStatePublished) {
      leakActive = false;
      mqttPublishLeak("OFF");
      leakStatePublished = true;
    }
  }
}

static void publishState() {
  unsigned long now = millis();

  mqttPublishRetained(totalGallonsTopic(), String(totalGallonsValue(), 3));

  const unsigned long windowMs = 10000UL;
  if (lastFlowCalcMs == 0) {
    lastFlowCalcMs = now;
    pulseCountAtLastFlowCalc = pulseCount;
  }

  unsigned long elapsed = now - lastFlowCalcMs;
  if (elapsed >= windowMs) {
    unsigned long pulsesDelta = pulseCount - pulseCountAtLastFlowCalc;
    float gallonsDelta = pulsesDelta * settings.gallonsPerPulse;
    float minutes = (float)elapsed / 60000.0f;
    float gpm = (minutes > 0.0f) ? (gallonsDelta / minutes) : 0.0f;

    flowGpmSmoothed = gpm;
    pulseCountAtLastFlowCalc = pulseCount;
    lastFlowCalcMs = now;
  }

  mqttPublishRetained(flowTopic(), String(flowGpmSmoothed, 3));
  updateLeakStatus(flowGpmSmoothed, now);

  if (WiFi.status() == WL_CONNECTED) {
    mqttPublishRetained(rssiTopic(), String(WiFi.RSSI()));
  }
}

static bool initSensorSPI() {
  pinMode(LIS3MDL_CS_PIN, OUTPUT);
  digitalWrite(LIS3MDL_CS_PIN, HIGH);

  SPI.begin();
  delay(10);
  yield();

  if (!lis.begin_SPI(LIS3MDL_CS_PIN)) return false;

  lis.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis.setRange(LIS3MDL_RANGE_4_GAUSS);

  return true;
}

static void appendInputField(String& body, const char* label, const char* name, const String& value, const char* type = "text", const char* step = nullptr, const char* hint = nullptr) {
  body += F("<label><span>");
  body += htmlEscape(label);
  body += F("</span><input name='");
  body += name;
  body += F("' type='");
  body += type;
  body += '\'';
  if (step && step[0] != '\0') {
    body += F(" step='");
    body += step;
    body += '\'';
  }
  body += F(" value='");
  body += htmlEscape(value);
  body += F("'></label>");
  if (hint && hint[0] != '\0') {
    body += F("<small>");
    body += htmlEscape(hint);
    body += F("</small>");
  }
}

static void appendCheckboxField(String& body, const char* label, const char* name, bool checked, const char* hint = nullptr) {
  body += F("<label class='checkbox'><input name='");
  body += name;
  body += F("' type='checkbox' value='1'");
  if (checked) body += F(" checked");
  body += F("><span>");
  body += htmlEscape(label);
  body += F("</span></label>");
  if (hint && hint[0] != '\0') {
    body += F("<small>");
    body += htmlEscape(hint);
    body += F("</small>");
  }
}

static String renderPage(const String& title, const String& body) {
  String page;
  page.reserve(body.length() + 2200);
  page += F("<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  page += F("<title>");
  page += htmlEscape(title);
  page += F("</title><style>");
  page += F("body{font-family:Segoe UI,Tahoma,sans-serif;background:#eef2f6;color:#17212b;margin:0;padding:24px;}");
  page += F(".shell{max-width:920px;margin:0 auto;background:#fff;border-radius:16px;padding:24px;box-shadow:0 12px 32px rgba(20,32,44,.12);}");
  page += F("h1,h2{margin:0 0 12px;}h2{margin-top:24px;font-size:1.05rem;border-top:1px solid #dde4ec;padding-top:18px;}");
  page += F(".meta{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px;margin:18px 0 24px;}");
  page += F(".card{background:#f6f9fc;border:1px solid #dde4ec;border-radius:12px;padding:12px 14px;}");
  page += F(".key{font-size:.78rem;text-transform:uppercase;letter-spacing:.06em;color:#5b6977;}.value{margin-top:6px;font-size:1rem;word-break:break-word;}");
  page += F("form{display:grid;gap:12px;}label{display:grid;gap:6px;}label span{font-weight:600;}input{font:inherit;padding:10px 12px;border-radius:10px;border:1px solid #c5d0db;}");
  page += F(".checkbox{display:flex;align-items:center;gap:10px;}.checkbox input{width:18px;height:18px;}");
  page += F("small{color:#60707f;margin-top:-6px;}button{font:inherit;padding:11px 14px;border:0;border-radius:10px;background:#0f6cbd;color:#fff;cursor:pointer;}");
  page += F(".actions{display:flex;flex-wrap:wrap;gap:12px;margin-top:20px;}.actions form{display:block;}");
  page += F(".secondary{background:#415a77;}.danger{background:#b42318;}a{color:#0f6cbd;text-decoration:none;}");
  page += F("</style></head><body><div class='shell'>");
  page += body;
  page += F("</div></body></html>");
  return page;
}

static String buildStatusBody() {
  String body;
  body.reserve(12000);

  body += F("<h1>Water Meter Console</h1><p>Editing settings for ");
  body += htmlEscape(settings.deviceName);
  body += F(" on the ");
  body += htmlEscape(settings.modelName);
  body += F(". Saving settings writes flash and reboots the device.</p>");

  body += F("<div class='meta'>");
  body += F("<div class='card'><div class='key'>Platform</div><div class='value'>");
  body += htmlEscape(PLATFORM_NAME);
  body += F("</div></div>");
  body += F("<div class='card'><div class='key'>IP Address</div><div class='value'>");
  body += (WiFi.status() == WL_CONNECTED) ? htmlEscape(WiFi.localIP().toString()) : String("Disconnected");
  body += F("</div></div>");
  body += F("<div class='card'><div class='key'>MQTT</div><div class='value'>");
  body += mqtt.connected() ? String("Connected") : String("Disconnected");
  body += F("</div></div>");
  body += F("<div class='card'><div class='key'>Sensor</div><div class='value'>");
  body += sensorAvailable ? String("Detected") : String("Missing");
  body += F("</div></div>");
  body += F("<div class='card'><div class='key'>Pulse Count</div><div class='value'>");
  body += String(pulseCount);
  body += F("</div></div>");
  body += F("<div class='card'><div class='key'>Total Gallons</div><div class='value'>");
  body += String(totalGallonsValue(), 3);
  body += F("</div></div>");
  body += F("<div class='card'><div class='key'>Flow</div><div class='value'>");
  body += String(flowGpmSmoothed, 3);
  body += F(" gpm</div></div></div>");

  body += F("<form method='post' action='/save'>");

  body += F("<h2>Network</h2>");
  appendInputField(body, "WiFi SSID", "wifi_ssid", settings.wifiSsid, "text", nullptr, "The wireless network this controller should join.");
  appendInputField(body, "WiFi Password", "wifi_pass", settings.wifiPass, "password", nullptr, "Password for the WiFi network above.");

  body += F("<h2>MQTT</h2>");
  appendInputField(body, "MQTT Host", "mqtt_host", settings.mqttHost, "text", nullptr, "IP address or hostname of your MQTT broker.");
  appendInputField(body, "MQTT Port", "mqtt_port", String(settings.mqttPort), "number", "1", "MQTT broker port, usually 1883.");
  appendInputField(body, "MQTT User", "mqtt_user", settings.mqttUser, "text", nullptr, "Username used when connecting to MQTT.");
  appendInputField(body, "MQTT Password", "mqtt_pass", settings.mqttPass, "password", nullptr, "Password used when connecting to MQTT.");
  appendInputField(body, "Base Topic", "base_topic", settings.baseTopic, "text", nullptr, "Starts on a test topic so it does not overwrite an existing meter until you change it.");
  appendInputField(body, "Home Assistant Prefix", "ha_prefix", settings.haPrefix, "text", nullptr, "Discovery prefix used by Home Assistant, usually homeassistant.");
  appendInputField(body, "Device ID", "device_id", settings.deviceId, "text", nullptr, "Stable unique ID used in MQTT discovery and entity identifiers.");
  appendInputField(body, "Device Name", "device_name", settings.deviceName, "text", nullptr, "Friendly display name shown in the UI and Home Assistant.");
  appendInputField(body, "Model Name", "model_name", settings.modelName, "text", nullptr, "Friendly hardware label shown in discovery metadata.");
  appendCheckboxField(body, "Enable MQTT RESET command on <base_topic>/cmd", "enable_remote_reset", settings.enableRemoteReset != 0, "Allows the counter to be reset remotely by publishing RESET to the command topic.");

  body += F("<h2>Meter Calibration</h2>");
  appendInputField(body, "Gallons Per Pulse", "gallons_per_pulse", String(settings.gallonsPerPulse, 5), "number", "0.0001", "How many gallons one meter pulse represents.");
  appendInputField(body, "Mag High Threshold", "mag_high", String(settings.magHigh, 2), "number", "0.1", "Magnetic magnitude must rise above this value to count a pulse.");
  appendInputField(body, "Mag Low Threshold", "mag_low", String(settings.magLow, 2), "number", "0.1", "Magnetic magnitude must fall below this value before the next pulse can arm.");
  appendInputField(body, "Pulse Lockout (ms)", "pulse_lockout_ms", String(settings.pulseLockoutMs), "number", "1", "Minimum time between accepted pulses. Lower values allow higher max GPM, but too low can double-count noise.");
  appendInputField(body, "Sensor Poll (ms)", "sensor_poll_ms", String(settings.sensorPollMs), "number", "1", "How often the LIS3MDL is sampled. Lower is faster but uses more CPU.");

  body += F("<h2>Publishing And Persistence</h2>");
  appendInputField(body, "Publish Interval (ms)", "publish_ms", String(settings.publishMs), "number", "1", "Normal MQTT publish rate when flow is low or idle.");
  appendInputField(body, "Fast Publish Interval (ms)", "fast_publish_ms", String(settings.fastPublishMs), "number", "1", "Faster MQTT publish rate used during active flow.");
  appendInputField(body, "Fast Flow Threshold (gpm)", "flow_fast_threshold", String(settings.flowFastThreshold, 3), "number", "0.001", "Flow rate that switches the device to the fast publish interval.");
  appendInputField(body, "Flash Save Interval (ms)", "persist_save_ms", String(settings.persistSaveMs), "number", "1", "How often pulse count is saved to flash. Shorter saves lose less on power loss but wear flash faster.");
  appendInputField(body, "Mag Debug Publish (ms)", "mag_debug_publish_ms", String(settings.magDebugPublishMs), "number", "1", "Set to 0 to disable magnetic debug MQTT publishes.");

  body += F("<h2>Safety Bounds</h2>");
  appendInputField(body, "Min Valid Magnetic Magnitude", "mag_min_valid", String(settings.magMinValid, 2), "number", "0.1", "Rejects unrealistically small readings that are probably bad samples.");
  appendInputField(body, "Max Valid Magnetic Magnitude", "mag_max_valid", String(settings.magMaxValid, 2), "number", "0.1", "Rejects unrealistically large spikes caused by noise or SPI glitches.");
  appendInputField(body, "Leak Minimum Flow (gpm)", "leak_min_gpm", String(settings.leakMinGpm, 3), "number", "0.001", "Flow must stay at or above this rate before leak timing starts.");
  appendInputField(body, "Leak Minimum Duration (ms)", "leak_min_duration_ms", String(settings.leakMinDurationMs), "number", "1", "How long flow must persist before the leak state turns on.");

  body += F("<button type='submit'>Save Settings And Reboot</button></form>");

  body += F("<h2>Firmware OTA</h2>");
  body += F("<form method='post' action='/update' enctype='multipart/form-data'>");
  body += F("<label><span>Firmware Binary</span><input name='firmware' type='file' accept='.bin'></label>");
  body += F("<small>Upload the matching firmware .bin for this board. The device will reboot automatically after a successful update.</small>");
  body += F("<button type='submit'>Upload Firmware</button></form>");

  body += F("<div class='actions'>");
  body += F("<form method='post' action='/reset-counter'><button class='secondary' type='submit'>Reset Pulse Counter</button></form>");
  body += F("<form method='post' action='/reboot'><button class='danger' type='submit'>Reboot Device</button></form>");
  body += F("</div>");

  return body;
}

static String numericArg(const char* name) {
  if (!webServer.hasArg(name)) return "";
  String value = webServer.arg(name);
  value.trim();
  return value;
}

static float floatArgOr(const char* name, float currentValue) {
  String value = numericArg(name);
  return value.length() ? value.toFloat() : currentValue;
}

static uint32_t uintArgOr(const char* name, uint32_t currentValue) {
  String value = numericArg(name);
  return value.length() ? (uint32_t)value.toInt() : currentValue;
}

static void applyPostedSettings() {
  if (webServer.hasArg("wifi_ssid")) copyText(settings.wifiSsid, webServer.arg("wifi_ssid"));
  if (webServer.hasArg("wifi_pass")) copyText(settings.wifiPass, webServer.arg("wifi_pass"));
  if (webServer.hasArg("mqtt_host")) copyText(settings.mqttHost, webServer.arg("mqtt_host"));
  if (webServer.hasArg("mqtt_user")) copyText(settings.mqttUser, webServer.arg("mqtt_user"));
  if (webServer.hasArg("mqtt_pass")) copyText(settings.mqttPass, webServer.arg("mqtt_pass"));
  if (webServer.hasArg("base_topic")) copyText(settings.baseTopic, webServer.arg("base_topic"));
  if (webServer.hasArg("ha_prefix")) copyText(settings.haPrefix, webServer.arg("ha_prefix"));
  if (webServer.hasArg("device_id")) copyText(settings.deviceId, webServer.arg("device_id"));
  if (webServer.hasArg("device_name")) copyText(settings.deviceName, webServer.arg("device_name"));
  if (webServer.hasArg("model_name")) copyText(settings.modelName, webServer.arg("model_name"));

  settings.mqttPort = (uint16_t)uintArgOr("mqtt_port", settings.mqttPort);
  settings.gallonsPerPulse = floatArgOr("gallons_per_pulse", settings.gallonsPerPulse);
  settings.magHigh = floatArgOr("mag_high", settings.magHigh);
  settings.magLow = floatArgOr("mag_low", settings.magLow);
  settings.pulseLockoutMs = uintArgOr("pulse_lockout_ms", settings.pulseLockoutMs);
  settings.sensorPollMs = uintArgOr("sensor_poll_ms", settings.sensorPollMs);
  settings.publishMs = uintArgOr("publish_ms", settings.publishMs);
  settings.fastPublishMs = uintArgOr("fast_publish_ms", settings.fastPublishMs);
  settings.flowFastThreshold = floatArgOr("flow_fast_threshold", settings.flowFastThreshold);
  settings.persistSaveMs = uintArgOr("persist_save_ms", settings.persistSaveMs);
  settings.magMinValid = floatArgOr("mag_min_valid", settings.magMinValid);
  settings.magMaxValid = floatArgOr("mag_max_valid", settings.magMaxValid);
  settings.leakMinGpm = floatArgOr("leak_min_gpm", settings.leakMinGpm);
  settings.leakMinDurationMs = uintArgOr("leak_min_duration_ms", settings.leakMinDurationMs);
  settings.magDebugPublishMs = uintArgOr("mag_debug_publish_ms", settings.magDebugPublishMs);
  settings.enableRemoteReset = webServer.hasArg("enable_remote_reset") ? 1 : 0;

  sanitizeSettings(settings);
}

static void scheduleRestart() {
  restartPending = true;
  restartScheduledMs = millis();
}

static void handleRoot() {
  webServer.send(200, "text/html", renderPage("Water Meter Console", buildStatusBody()));
}

static void handleSave() {
  applyPostedSettings();
  saveState(true);

  String body;
  body += F("<h1>Settings Saved</h1><p>The device will reboot in a moment so the new WiFi, MQTT, and calibration settings are applied.</p>");
  body += F("<p><a href='/'>Return to the WebUI</a></p>");
  webServer.send(200, "text/html", renderPage("Settings Saved", body));

  scheduleRestart();
}

static void handleResetCounter() {
  resetMeterCount();
  publishState();

  String body;
  body += F("<h1>Counter Reset</h1><p>The pulse counter was cleared and the updated state was published to MQTT if connected.</p>");
  body += F("<p><a href='/'>Return to the WebUI</a></p>");
  webServer.send(200, "text/html", renderPage("Counter Reset", body));
}

static void handleReboot() {
  String body;
  body += F("<h1>Rebooting</h1><p>The device will restart in a moment.</p><p><a href='/'>Return to the WebUI</a></p>");
  webServer.send(200, "text/html", renderPage("Rebooting", body));
  scheduleRestart();
}

static void handleFirmwareUpdateUpload() {
  HTTPUpload& upload = webServer.upload();

  if (upload.status == UPLOAD_FILE_START) {
    otaUpdateStarted = true;
    otaUpdateSuccess = false;
    otaUpdateError = "";

    Serial.printf("OTA upload starting: %s\n", upload.filename.c_str());
    saveState(true);
    mqttPublishStatus("updating");
    mqtt.disconnect();

    if (!beginFirmwareUpdate()) {
      otaUpdateError = firmwareUpdateErrorString();
      Serial.printf("OTA begin failed: %s\n", otaUpdateError.c_str());
    }
    return;
  }

  if (!otaUpdateStarted) return;

  if (upload.status == UPLOAD_FILE_WRITE) {
    if (otaUpdateError.length() > 0) return;

    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      otaUpdateError = firmwareUpdateErrorString();
      Serial.printf("OTA write failed: %s\n", otaUpdateError.c_str());
    }
    return;
  }

  if (upload.status == UPLOAD_FILE_END) {
    if (otaUpdateError.length() == 0) {
      if (Update.end(true)) {
        otaUpdateSuccess = true;
        Serial.printf("OTA upload complete: %u bytes\n", upload.totalSize);
      } else {
        otaUpdateError = firmwareUpdateErrorString();
        Serial.printf("OTA finalize failed: %s\n", otaUpdateError.c_str());
      }
    } else {
      cancelFirmwareUpdate();
    }
    return;
  }

  if (upload.status == UPLOAD_FILE_ABORTED) {
    otaUpdateError = "Upload aborted";
    cancelFirmwareUpdate();
    Serial.println("OTA upload aborted");
  }
}

static void handleFirmwareUpdatePost() {
  String body;

  if (!otaUpdateStarted) {
    body += F("<h1>Firmware Upload Failed</h1><p>No firmware file was received.</p>");
    body += F("<p><a href='/'>Return to the WebUI</a></p>");
    webServer.send(400, "text/html", renderPage("Firmware Upload Failed", body));
    return;
  }

  if (otaUpdateSuccess) {
    body += F("<h1>Firmware Updated</h1><p>The firmware image was written successfully. The device will reboot in a moment.</p>");
    body += F("<p><a href='/'>Return to the WebUI</a></p>");
    webServer.sendHeader("Connection", "close");
    webServer.send(200, "text/html", renderPage("Firmware Updated", body));
    scheduleRestart();
  } else {
    body += F("<h1>Firmware Upload Failed</h1><p>");
    body += htmlEscape(otaUpdateError.length() > 0 ? otaUpdateError : String("Unknown update error"));
    body += F("</p><p><a href='/'>Return to the WebUI</a></p>");
    webServer.send(500, "text/html", renderPage("Firmware Upload Failed", body));
  }

  otaUpdateStarted = false;
  otaUpdateSuccess = false;
}

static void handleStatusApi() {
  StaticJsonDocument<384> doc;
  doc["device"] = settings.deviceName;
  doc["platform"] = PLATFORM_NAME;
  doc["ip"] = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "disconnected";
  doc["mqtt"] = mqtt.connected();
  doc["sensor_available"] = sensorAvailable;
  doc["pulse_count"] = pulseCount;
  doc["total_gallons"] = totalGallonsValue();
  doc["flow_gpm"] = flowGpmSmoothed;

  String payload;
  serializeJson(doc, payload);
  webServer.send(200, "application/json", payload);
}

static void handleNotFound() {
  webServer.send(404, "text/plain", "Not found");
}

static void setupWebServer() {
  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/save", HTTP_POST, handleSave);
  webServer.on("/update", HTTP_POST, handleFirmwareUpdatePost, handleFirmwareUpdateUpload);
  webServer.on("/reset-counter", HTTP_POST, handleResetCounter);
  webServer.on("/reboot", HTTP_POST, handleReboot);
  webServer.on("/api/status", HTTP_GET, handleStatusApi);
  webServer.onNotFound(handleNotFound);
  webServer.begin();
}

void waterMeterSetup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("Booting water meter firmware");

  sensorAvailable = initSensorSPI();
  if (!sensorAvailable) {
    Serial.println("LIS3MDL not detected. Continuing without sensor so WebUI and OTA remain available.");
  } else {
    Serial.println("LIS3MDL detected");
  }

  loadState();
  Serial.printf("Restored pulse count: %lu\n", pulseCount);

  applyWifiClientSettings();
  mqtt.setBufferSize(MQTT_BUFFER_BYTES);
  mqtt.setCallback(mqttCallback);

  setupWebServer();
  wifiEnsureConnected();
  mqttEnsureConnected();
  publishState();
  lastPublishMs = millis();

  Serial.print("WebUI ready on hostname ");
  Serial.println(settings.deviceId);
}

void waterMeterLoop() {
  wifiEnsureConnected();
  logWifiTransitions();

  mqttEnsureConnected();
  mqtt.loop();
  webServer.handleClient();

  unsigned long now = millis();

  if (sensorAvailable && (now - lastSensorPollMs >= settings.sensorPollMs)) {
    lastSensorPollMs = now;

    sensors_event_t event;
    if (lis.getEvent(&event)) {
      float mag = sqrtf(
        event.magnetic.x * event.magnetic.x +
        event.magnetic.y * event.magnetic.y +
        event.magnetic.z * event.magnetic.z
      );

      if (settings.magDebugPublishMs > 0 && (now - lastMagDebugPublishMs) >= settings.magDebugPublishMs) {
        lastMagDebugPublishMs = now;
        mqttPublishRetained(magDebugTopic(), String(mag, 1));
      }

      if (isfinite(mag) && mag > settings.magMinValid && mag < settings.magMaxValid) {
        if (!triggered && mag > settings.magHigh && (now - lastPulseAcceptedMs) > settings.pulseLockoutMs) {
          pulseCount++;
          lastPulseMs = now;
          lastPulseAcceptedMs = now;
          triggered = true;

          saveState(false);
          Serial.printf("Pulse %lu |Mag|=%.2f\n", pulseCount, mag);
        }

        if (triggered && mag < settings.magLow) {
          triggered = false;
        }
      }
    }
  }

  unsigned long publishInterval = (flowGpmSmoothed > settings.flowFastThreshold) ? settings.fastPublishMs : settings.publishMs;
  if (now - lastPublishMs >= publishInterval) {
    lastPublishMs = now;
    publishState();
    saveState(false);
  }

  if (restartPending && (now - restartScheduledMs) >= RESTART_DELAY_MS) {
    saveState(true);
    ESP.restart();
  }

  yield();
}
