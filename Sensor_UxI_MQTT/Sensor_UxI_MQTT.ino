/*
  ============================================
  Lightweight UxI MQTT Sensor for ESP32
  ============================================
  
  Remote measurement node that reads voltage and current
  via UxI method and publishes to MQTT broker.
  
  Designed to work with Solar_Router as a remote sensor.
  
  Hardware:
  - ESP32 (any variant)
  - Voltage transformer (3.3V secondary)
  - Current sensor (ACS712 or similar)
  - Analog inputs: GPIO34 (voltage), GPIO32 (CT 1), GPIO35 (CT 2)
  - GPIO33: shared current sensor reference
  - Both CTs measure circuits on the same phase as the voltage sensor.
  - Voltage phase correction: -0.3 degrees (lag) for both power calculations.

  OTA updates:
  - Set ota_password below; OTA stays disabled while it is empty.
  - Upload once by USB using a partition scheme that supports OTA.
  - Subsequent uploads: select uxi-sensor in Arduino IDE's network ports.

  ESP-NOW:
  - Broadcast all measurements every 100 ms using EspNowMeasurements.h.
  - Receiver must use the sender's current WiFi channel.
  - Use the AP channel when connected, or ESPNOW_OFFLINE_CHANNEL when offline.
  - Broadcasts are unencrypted and have no delivery acknowledgement.
  - Interval is best-effort; WiFi/MQTT reconnection and OTA can delay sends.
  - Protocol v2 includes the sender's current WiFi channel; update both boards.

  WiFi:
  - Measurements, energy persistence and ESP-NOW run without an available AP.
  - Non-blocking connection attempts start every 30 seconds, timing out after 10.
  - Connection scans can briefly interrupt ESP-NOW reception.
  - MQTT and OTA are serviced only while WiFi is connected.
  
  MQTT Topics Published:
  - Channel suffix: _1 for GPIO32, _2 for GPIO35
  - solar/sensor/power_1, power_2 (Consumption, Watts, nonnegative)
  - solar/sensor/injection_1, injection_2 (Injection, Watts, nonnegative)
  - solar/sensor/current_1, current_2 (Amps RMS)
  - solar/sensor/pva_1, pva_2 (Apparent Power, VA)
  - solar/sensor/pf_1, pf_2 (Power Factor magnitude in either direction)
  - solar/sensor/energy_1, energy_2 (Consumed Wh total)
  - solar/sensor/injection_energy_1, injection_energy_2 (Injected Wh total)
  - solar/sensor/voltage (Volts RMS)
  - Unsuffixed power/current/pva/pf/energy remain aliases for channel 1.
  - Energy uses the nominal 40 ms measurement interval.
  - All four energy counters are restored from NVS and saved every 20 minutes.
  - A reboot or power loss can discard energy accumulated since the last save.
  - Home Assistant MQTT discovery under homeassistant/sensor/
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <cmath>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include "EspNowMeasurements.h"

// ========== CONFIGURATION ==========
const char* ssid = "MatMat_2G";
const char* password = "WelcomeSuresnes!";
const char* mqtt_server = "10.3.0.1";  // IP of your MQTT broker
const int mqtt_port = 1883;
const char* mqtt_user = "pi";
const char* mqtt_password = "Jcubf434";
const char* mqtt_client_id = "UxI_Sensor_01";
const char* mqtt_topic_prefix = "solar/sensor";
const char* mqtt_discovery_prefix = "homeassistant";
const char* mqtt_device_name = "UxI MQTT Sensor";
const char* ota_hostname = "uxi-sensor";
const char* ota_password = "jcubf43";  // Set a strong password to enable OTA
bool otaStarted = false;

// Analog input pins
const int AnalogIn0 = 33;  // Current sensor reference voltage
const int AnalogIn1 = 34;  // Voltage input
const int AnalogIn2 = 32;  // CT 1 current input
const int AnalogIn3 = 35;  // CT 2 current input

// Calibration constants - ADJUST THESE FOR YOUR SETUP
// kV: voltage calibration (depends on transformer ratio and ADC resolution)
// kI1, kI2: independent current calibration for each CT
double kV = 0.7131086142;  // Calibrated from 267 V indicated to 238 V measured
const double voltagePhaseCorrectionDegrees = -15;  // Negative delays voltage; RMS is unchanged
double kI1 = 0.0501687243686;
double kI2 = 0.0165868112754; 

// Measurement arrays
int volt[100], amp1[100], amp2[100];
double voltM[100], ampM1[100], ampM2[100];

// Power variables
double Tension_M = 0;
double Current_1 = 0;
double Current_2 = 0;
double Power_1 = 0;
double Power_2 = 0;
double Injection_1 = 0;
double Injection_2 = 0;
double PVA_1 = 0;
double PVA_2 = 0;
double PowerFactor_1 = 0;
double PowerFactor_2 = 0;
double Energy_1 = 0;
double Energy_2 = 0;
double InjectionEnergy_1 = 0;
double InjectionEnergy_2 = 0;

Preferences energyPreferences;
bool energyStorageReady = false;

// MQTT
WiFiClient espClient;
PubSubClient client(espClient);

bool wifiReady = false;
bool wifiConnecting = false;
bool wifiWasConnected = false;
bool wifiAttempted = false;

const uint8_t espNowBroadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const uint8_t ESPNOW_OFFLINE_CHANNEL = 3;
bool espNowReady = false;
uint32_t espNowSequence = 0;

// Timing
unsigned long lastMeasure = 0;
unsigned long lastPublish = 0;
unsigned long lastMqttAttempt = 0;
unsigned long lastWiFiAttempt = 0;
unsigned long lastEnergySave = 0;
unsigned long lastEspNowPublish = 0;
bool mqttAttempted = false;
const unsigned long MEASURE_INTERVAL = 40;  // Match the router's 25 Hz measurement rate
const unsigned long PUBLISH_INTERVAL = 15000;  // Publish every 15 seconds
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;
const unsigned long WIFI_RECONNECT_INTERVAL = 30000;
const unsigned long WIFI_CONNECT_TIMEOUT = 10000;
const unsigned long ENERGY_SAVE_INTERVAL = 20UL * 60UL * 1000UL;
const unsigned long ESPNOW_PUBLISH_INTERVAL = 100;
char availability_topic[100];

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=== UxI MQTT Sensor Starting ===");
  
  Setup_UxI();
  restoreEnergyCounters();
  lastEnergySave = millis();
  setupWiFi();
  setupEspNow();
  client.setServer(mqtt_server, mqtt_port);
  client.setSocketTimeout(1);
  client.setBufferSize(768);
  snprintf(availability_topic, sizeof(availability_topic), "%s/availability", mqtt_topic_prefix);
  analogSetAttenuation(ADC_11db);

  pinMode(AnalogIn0, INPUT);
  pinMode(AnalogIn1, INPUT);
  pinMode(AnalogIn2, INPUT);
  pinMode(AnalogIn3, INPUT);
  Serial.println("Setup complete!");
}

// ========== MAIN LOOP ==========
void loop() {
  updateWiFi();
  const bool wifiConnected = WiFi.isConnected();

  if (wifiConnected && !otaStarted && ota_password[0] != '\0') {
    setupOTA();
  }
  if (otaStarted && wifiConnected) {
    ArduinoOTA.handle();
  }

  if (wifiConnected && !client.connected() &&
      (!mqttAttempted || millis() - lastMqttAttempt >= MQTT_RECONNECT_INTERVAL)) {
    mqttAttempted = true;
    lastMqttAttempt = millis();
    reconnect();
  }
  if (wifiConnected && client.connected()) {
    client.loop();
  }
  
  // Measure power every MEASURE_INTERVAL
  if (millis() - lastMeasure >= MEASURE_INTERVAL) {
    lastMeasure = millis();
    LectureUxI();
  }
  
  // Publish to MQTT every PUBLISH_INTERVAL
  if (wifiConnected && client.connected() && millis() - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = millis();
    publishMQTT();
  }

  if (espNowReady && millis() - lastEspNowPublish >= ESPNOW_PUBLISH_INTERVAL) {
    lastEspNowPublish = millis();
    publishEspNow();
  }
  
  if (millis() - lastEnergySave >= ENERGY_SAVE_INTERVAL) {
    lastEnergySave = millis();
    saveEnergyCounters();
  }

  delay(10);
}

// ========== ESP-NOW ==========
void setupEspNow() {
  esp_err_t result = esp_now_init();
  if (result != ESP_OK) {
    Serial.printf("ESP-NOW initialization failed: %s\n", esp_err_to_name(result));
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, espNowBroadcastAddress, sizeof(peer.peer_addr));
  peer.channel = 0;  // Follow the AP channel or the offline channel.
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  result = esp_now_add_peer(&peer);
  if (result != ESP_OK) {
    Serial.printf("ESP-NOW broadcast peer setup failed: %s\n", esp_err_to_name(result));
    result = esp_now_deinit();
    if (result != ESP_OK) {
      Serial.printf("ESP-NOW cleanup failed: %s\n", esp_err_to_name(result));
    }
    return;
  }

  espNowReady = true;
  Serial.printf("ESP-NOW broadcast ready on WiFi channel %d\n", WiFi.channel());
}

void publishEspNow() {
  uint8_t primaryChannel = 0;
  wifi_second_chan_t secondaryChannel;
  esp_err_t result = esp_wifi_get_channel(&primaryChannel, &secondaryChannel);
  if (result != ESP_OK) {
    Serial.printf("ESP-NOW broadcast skipped: cannot read WiFi channel: %s\n", esp_err_to_name(result));
    return;
  }

  EspNowMeasurements packet = {};
  packet.magic = ESPNOW_MEASUREMENTS_MAGIC;
  packet.version = ESPNOW_MEASUREMENTS_VERSION;
  packet.size = sizeof(packet);
  packet.sequence = espNowSequence++;
  packet.uptimeMillis = millis();
  packet.wifiChannel = primaryChannel;
  packet.voltage = Tension_M;
  packet.channels[0] = {Current_1, Power_1, Injection_1, PVA_1,
                        PowerFactor_1, Energy_1, InjectionEnergy_1};
  packet.channels[1] = {Current_2, Power_2, Injection_2, PVA_2,
                        PowerFactor_2, Energy_2, InjectionEnergy_2};

  result = esp_now_send(espNowBroadcastAddress,
                        reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
  if (result != ESP_OK) {
    Serial.printf("ESP-NOW broadcast could not be queued: %s\n", esp_err_to_name(result));
  }
}

// ========== ENERGY PERSISTENCE ==========
void restoreEnergyCounters() {
  if (!energyPreferences.begin("uxi-energy", false)) {
    Serial.println("Failed to open energy storage; persistence disabled");
    return;
  }
  if (!energyPreferences.isKey("totals")) {
    energyStorageReady = true;
    Serial.println("No saved energy counters; starting at zero");
    return;
  }

  double totals[4];
  if (energyPreferences.getBytesLength("totals") != sizeof(totals) ||
      energyPreferences.getBytes("totals", totals, sizeof(totals)) != sizeof(totals)) {
    Serial.println("Failed to read energy counters; persistence disabled to preserve stored data");
    energyPreferences.end();
    return;
  }
  for (double value : totals) {
    if (!std::isfinite(value) || value < 0) {
      Serial.println("Invalid saved energy counters; persistence disabled to preserve stored data");
      energyPreferences.end();
      return;
    }
  }

  Energy_1 = totals[0];
  Energy_2 = totals[1];
  InjectionEnergy_1 = totals[2];
  InjectionEnergy_2 = totals[3];
  energyStorageReady = true;
  Serial.println("Energy counters restored from NVS");
}

void saveEnergyCounters() {
  if (!energyStorageReady) {
    Serial.println("Energy counters not saved: storage unavailable");
    return;
  }

  // One NVS blob keeps all four counters in the same committed snapshot.
  const double totals[] = {Energy_1, Energy_2, InjectionEnergy_1, InjectionEnergy_2};
  for (double value : totals) {
    if (!std::isfinite(value) || value < 0) {
      Serial.println("Invalid energy counters; keeping the previous NVS snapshot");
      return;
    }
  }
  if (energyPreferences.putBytes("totals", totals, sizeof(totals)) != sizeof(totals)) {
    Serial.println("Failed to save energy counters; will retry at the next 20-minute interval");
    return;
  }
  Serial.println("Energy counters saved to NVS");
}

// ========== OTA SETUP ==========
void setupOTA() {
  if (otaStarted) {
    return;
  }
  if (ota_password[0] == '\0') {
    Serial.println("OTA disabled: configure ota_password to enable updates");
    return;
  }

  ArduinoOTA.setHostname(ota_hostname);
  ArduinoOTA.setPassword(ota_password);
  ArduinoOTA.onStart([]() {
    Serial.println("OTA update starting");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA update complete; restarting");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static unsigned int lastPercent = 101;
    unsigned int percent = total > 0 ? uint64_t(progress) * 100 / total : 0;
    if (percent != lastPercent) {
      Serial.printf("OTA progress: %u%%\n", percent);
      lastPercent = percent;
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA failed (error %u)\n", static_cast<unsigned int>(error));
  });
  ArduinoOTA.begin();
  otaStarted = true;
  Serial.printf("OTA ready: %s.local\n", ota_hostname);
}

// ========== WIFI SETUP ==========
void setOfflineWiFiChannel() {
  esp_err_t result = esp_wifi_set_channel(ESPNOW_OFFLINE_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (result != ESP_OK) {
    Serial.printf("Failed to set offline ESP-NOW channel: %s\n", esp_err_to_name(result));
    return;
  }
  Serial.printf("Offline ESP-NOW channel: %u\n", ESPNOW_OFFLINE_CHANNEL);
}

void setupWiFi() {
  if (!WiFi.mode(WIFI_STA)) {
    Serial.println("Failed to start WiFi station; continuing local measurements");
    return;
  }
  if (!WiFi.setAutoReconnect(false)) {
    Serial.println("Failed to disable automatic WiFi reconnect; continuing local measurements");
    return;
  }
  wifiReady = true;
  setOfflineWiFiChannel();
}

void updateWiFi() {
  if (!wifiReady) {
    return;
  }
  if (WiFi.isConnected()) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      wifiConnecting = false;
      wifiAttempted = false;
      Serial.println("WiFi connected!");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    }
    return;
  }

  if (wifiWasConnected) {
    wifiWasConnected = false;
    Serial.println("WiFi lost; continuing measurements and ESP-NOW offline");
    setOfflineWiFiChannel();
  }
  if (wifiConnecting) {
    if (millis() - lastWiFiAttempt < WIFI_CONNECT_TIMEOUT) {
      return;
    }
    wifiConnecting = false;
    Serial.printf("WiFi connection timed out (status %d); continuing offline\n", WiFi.status());
    if (!WiFi.disconnect(false, false)) {
      Serial.println("Failed to stop WiFi connection attempt");
    }
    setOfflineWiFiChannel();
  }

  if (!wifiAttempted || millis() - lastWiFiAttempt >= WIFI_RECONNECT_INTERVAL) {
    wifiAttempted = true;
    wifiConnecting = true;
    lastWiFiAttempt = millis();
    Serial.println("Starting non-blocking WiFi connection attempt");
    WiFi.begin(ssid, password);
  }
}

// ========== MQTT RECONNECT ==========
void reconnect() {
  if (client.connect(mqtt_client_id, mqtt_user, mqtt_password,
                     availability_topic, 0, true, "offline")) {
    Serial.println("MQTT connected!");
    if (!client.publish(availability_topic, "online", true)) {
      Serial.println("Failed to publish MQTT availability");
    }
    publishHomeAssistantDiscovery();
  } else {
    Serial.print("MQTT connection failed, rc=");
    Serial.println(client.state());
  }
}

// ========== HOME ASSISTANT MQTT DISCOVERY ==========
void publishDiscoverySensor(const char* object_id, const char* name,
                            const char* state_suffix, const char* unit,
                            const char* device_class, const char* state_class) {
  char config_topic[160];
  char state_topic[100];
  char unique_id[100];
  char metadata[160] = "";
  char payload[600];

  snprintf(config_topic, sizeof(config_topic), "%s/sensor/%s/%s/config",
           mqtt_discovery_prefix, mqtt_client_id, object_id);
  snprintf(state_topic, sizeof(state_topic), "%s/%s", mqtt_topic_prefix, state_suffix);
  snprintf(unique_id, sizeof(unique_id), "%s_%s", mqtt_client_id, object_id);

  if (unit[0] != '\0') {
    snprintf(metadata + strlen(metadata), sizeof(metadata) - strlen(metadata),
             ",\"unit_of_measurement\":\"%s\"", unit);
  }
  if (device_class[0] != '\0') {
    snprintf(metadata + strlen(metadata), sizeof(metadata) - strlen(metadata),
             ",\"device_class\":\"%s\"", device_class);
  }
  if (state_class[0] != '\0') {
    snprintf(metadata + strlen(metadata), sizeof(metadata) - strlen(metadata),
             ",\"state_class\":\"%s\"", state_class);
  }

  snprintf(payload, sizeof(payload),
           "{\"name\":\"%s\",\"unique_id\":\"%s\",\"state_topic\":\"%s\","
           "\"availability_topic\":\"%s\",\"payload_available\":\"online\","
           "\"payload_not_available\":\"offline\",\"device\":{\"identifiers\":[\"%s\"],"
           "\"name\":\"%s\",\"manufacturer\":\"F1ATB\",\"model\":\"ESP32 UxI Sensor\"}%s}",
           name, unique_id, state_topic, availability_topic, mqtt_client_id,
           mqtt_device_name, metadata);

  if (!client.publish(config_topic, payload, true)) {
    Serial.print("Failed to publish Home Assistant discovery for ");
    Serial.println(object_id);
  }
}

void publishHomeAssistantDiscovery() {
  publishDiscoverySensor("power", "Power_1", "power_1", "W", "power", "measurement");
  publishDiscoverySensor("power_2", "Power_2", "power_2", "W", "power", "measurement");
  publishDiscoverySensor("injection_1", "Injection_1", "injection_1", "W", "power", "measurement");
  publishDiscoverySensor("injection_2", "Injection_2", "injection_2", "W", "power", "measurement");
  publishDiscoverySensor("voltage", "UxI Voltage", "voltage", "V", "voltage", "measurement");
  publishDiscoverySensor("current", "Current_1", "current_1", "A", "current", "measurement");
  publishDiscoverySensor("current_2", "Current_2", "current_2", "A", "current", "measurement");
  publishDiscoverySensor("power_factor", "Power Factor_1", "pf_1", "", "", "measurement");
  publishDiscoverySensor("power_factor_2", "Power Factor_2", "pf_2", "", "", "measurement");
  publishDiscoverySensor("apparent_power", "Apparent Power_1", "pva_1", "VA", "apparent_power", "measurement");
  publishDiscoverySensor("apparent_power_2", "Apparent Power_2", "pva_2", "VA", "apparent_power", "measurement");
  publishDiscoverySensor("energy", "Energy_1", "energy_1", "Wh", "energy", "total_increasing");
  publishDiscoverySensor("energy_2", "Energy_2", "energy_2", "Wh", "energy", "total_increasing");
  publishDiscoverySensor("injection_energy_1", "Injection Energy_1", "injection_energy_1", "Wh", "energy", "total_increasing");
  publishDiscoverySensor("injection_energy_2", "Injection Energy_2", "injection_energy_2", "Wh", "energy", "total_increasing");
}

// ========== PUBLISH TO MQTT ==========
void publishMeasurement(const char* suffix, double value, int decimals) {
  char topic[100];
  char payload[20];
  snprintf(topic, sizeof(topic), "%s/%s", mqtt_topic_prefix, suffix);
  snprintf(payload, sizeof(payload), "%.*f", decimals, value);
  if (!client.publish(topic, payload, true)) {
    Serial.print("Failed to publish ");
    Serial.println(suffix);
  }
}

void publishMQTT() {
  publishMeasurement("voltage", Tension_M, 2);

  publishMeasurement("power_1", Power_1, 1);
  publishMeasurement("injection_1", Injection_1, 1);
  publishMeasurement("current_1", Current_1, 3);
  publishMeasurement("pva_1", PVA_1, 1);
  publishMeasurement("pf_1", PowerFactor_1, 2);
  publishMeasurement("energy_1", Energy_1, 1);
  publishMeasurement("injection_energy_1", InjectionEnergy_1, 1);

  publishMeasurement("power_2", Power_2, 1);
  publishMeasurement("injection_2", Injection_2, 1);
  publishMeasurement("current_2", Current_2, 3);
  publishMeasurement("pva_2", PVA_2, 1);
  publishMeasurement("pf_2", PowerFactor_2, 2);
  publishMeasurement("energy_2", Energy_2, 1);
  publishMeasurement("injection_energy_2", InjectionEnergy_2, 1);

  publishMeasurement("power", Power_1, 1);
  publishMeasurement("current", Current_1, 3);
  publishMeasurement("pva", PVA_1, 1);
  publishMeasurement("pf", PowerFactor_1, 2);
  publishMeasurement("energy", Energy_1, 1);

  Serial.printf("Published: U=%.2fV\n", Tension_M);
  Serial.printf("Channel 1: P=%.1fW, Injection=%.1fW, I=%.3fA, S=%.1fVA, PF=%.2f, E=%.1fWh, Injected E=%.1fWh\n",
                Power_1, Injection_1, Current_1, PVA_1, PowerFactor_1, Energy_1, InjectionEnergy_1);
  Serial.printf("Channel 2: P=%.1fW, Injection=%.1fW, I=%.3fA, S=%.1fVA, PF=%.2f, E=%.1fWh, Injected E=%.1fWh\n",
                Power_2, Injection_2, Current_2, PVA_2, PowerFactor_2, Energy_2, InjectionEnergy_2);
}

// ========== UxI MEASUREMENT FUNCTIONS ==========
void Setup_UxI() {
  for (int i = 0; i < 100; i++) {
    voltM[i] = 0;
    ampM1[i] = 0;
    ampM2[i] = 0;
  }
  Serial.println("UxI measurement initialized");
}

void LectureUxI() {
  MeasurePower();
  ComputePower();
}

void MeasurePower() {
  int iStore;
  int currentReference = analogRead(AnalogIn0);
  static unsigned long OverflowOffset = 0;
  static unsigned long PrevMicros = 0;
  
  unsigned long NowMicros;
  unsigned long MeasureMillis = millis();
  
  // Read values during 20ms
  while (millis() - MeasureMillis < 21) {
    NowMicros = micros();
    
    if (NowMicros < PrevMicros) {
      OverflowOffset = (OverflowOffset + 7296) % 20000;
    }
    
    iStore = ((NowMicros % 20000 + OverflowOffset) % 20000) / 200;
    volt[iStore] = analogRead(AnalogIn1);
    amp1[iStore] = analogRead(AnalogIn2) - currentReference;
    amp2[iStore] = analogRead(AnalogIn3) - currentReference;
    
    PrevMicros = NowMicros;
  }
}

void ComputePower() {
  double PWcal1 = 0;
  double PWcal2 = 0;
  double V;
  double I1;
  double I2;
  double Uef2 = 0;
  double Ief1Squared = 0;
  double Ief2Squared = 0;
  double voltageOffset = 0;
  double currentOffset1 = 0;
  double currentOffset2 = 0;
  
  for (int i = 0; i < 100; i++) {
    voltM[i] = (19.0 * voltM[i] + double(volt[i])) / 20.0;
    ampM1[i] = (19.0 * ampM1[i] + double(amp1[i])) / 20.0;
    ampM2[i] = (19.0 * ampM2[i] + double(amp2[i])) / 20.0;
    voltageOffset += voltM[i];
    currentOffset1 += ampM1[i];
    currentOffset2 += ampM2[i];
  }

  voltageOffset /= 100.0;
  currentOffset1 /= 100.0;
  currentOffset2 /= 100.0;

  // 100 samples span 360 degrees: -0.3 degrees is a delay of 1/12 sample.
  const double voltageSampleShift = voltagePhaseCorrectionDegrees * 100.0 / 360.0;
  // floor(-1/12) = -1; the remaining fraction is 11/12.
  const int voltageIndexShift = int(floor(voltageSampleShift));
  const double voltageFraction = voltageSampleShift - voltageIndexShift;

  for (int i = 0; i < 100; i++) {
    V = kV * (voltM[i] - voltageOffset);
    // Wrap within the mains cycle, including sample 99 before sample 0.
    const int voltageIndex = ((i + voltageIndexShift) % 100 + 100) % 100;
    const int nextVoltageIndex = (voltageIndex + 1) % 100;
    // At -0.3 degrees: 1/12 of the previous sample + 11/12 of the current sample.
    const double shiftedVoltage = (1.0 - voltageFraction) * voltM[voltageIndex]
                                  + voltageFraction * voltM[nextVoltageIndex];
    const double correctedV = kV * (shiftedVoltage - voltageOffset);
    I1 = kI1 * (ampM1[i] - currentOffset1);
    I2 = kI2 * (ampM2[i] - currentOffset2);
    // Keep RMS voltage unshifted; only real power uses the phase-corrected voltage.
    Uef2 += sq(V);
    Ief1Squared += sq(I1);
    Ief2Squared += sq(I2);
    PWcal1 += correctedV * I1;
    PWcal2 += correctedV * I2;
  }
  
  Uef2 = Uef2 / 100.0;
  Tension_M = sqrt(Uef2);
  
  Current_1 = sqrt(Ief1Squared / 100.0);
  Current_2 = sqrt(Ief2Squared / 100.0);

  Power_1 = max(0.0, PWcal1 / 100.0);
  Power_2 = max(0.0, PWcal2 / 100.0);
  Injection_1 = max(0.0, -PWcal1 / 100.0);
  Injection_2 = max(0.0, -PWcal2 / 100.0);
  PVA_1 = Tension_M * Current_1;
  PVA_2 = Tension_M * Current_2;

  PowerFactor_1 = PVA_1 > 0 ? floor(100.0 * (Power_1 + Injection_1) / PVA_1) / 100.0 : 0;
  PowerFactor_2 = PVA_2 > 0 ? floor(100.0 * (Power_2 + Injection_2) / PVA_2) / 100.0 : 0;

  Energy_1 += Power_1 * MEASURE_INTERVAL / 3600000.0;
  Energy_2 += Power_2 * MEASURE_INTERVAL / 3600000.0;
  InjectionEnergy_1 += Injection_1 * MEASURE_INTERVAL / 3600000.0;
  InjectionEnergy_2 += Injection_2 * MEASURE_INTERVAL / 3600000.0;
}
