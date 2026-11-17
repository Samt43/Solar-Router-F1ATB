/*
  ESP32 UxI ESP-NOW receiver

  Automatically scan the configured WiFi country's channels, dwelling 1.5 s each.
  Lock on a valid measurement packet; resume scanning after 5 s without one.
  Open Serial Monitor at 115200 baud to see received measurements.
  Keep EspNowMeasurements.h in this sketch folder, matching the sender's header.
  Protocol v2 logs sender and receiver WiFi channels; update both boards.
  No WiFi credentials or ESP-NOW peers are required for unencrypted broadcasts.
  The onboard LED pulses for about 100 ms per message, timed by a hardware ISR.
  Adjust its pin/polarity below.
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include <esp_idf_version.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <cstring>
#include "EspNowMeasurements.h"

constexpr uint8_t ESPNOW_START_CHANNEL = 3;  // First channel to try, not a fixed channel
constexpr unsigned long CHANNEL_DWELL_INTERVAL = 1500;
constexpr unsigned long SIGNAL_LOSS_TIMEOUT = 5000;
constexpr uint8_t STATUS_LED_PIN = 2;  // Common classic ESP32 DevKit onboard LED
constexpr bool STATUS_LED_ACTIVE_LOW = false;
constexpr unsigned long LED_PULSE_DURATION = 100;  // Milliseconds

struct ReceivedMeasurements {
  uint8_t sender[6];
  int length;
  uint8_t channel;
  esp_err_t channelReadResult;
  unsigned long receivedAt;
  EspNowMeasurements packet;
};

// Keep Arduino's generated prototype from preceding this sketch-local type.
void logPacket(const ReceivedMeasurements& received);

QueueHandle_t receiveQueue = nullptr;
bool receiverReady = false;
bool channelLocked = false;
uint8_t firstScanChannel = 0;
uint8_t lastScanChannel = 0;
uint8_t currentChannel = 0;
unsigned long lastChannelAttempt = 0;
unsigned long lastValidPacketAt = 0;
portMUX_TYPE dropCountMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t droppedPackets = 0;
hw_timer_t* ledTimer = nullptr;
portMUX_TYPE ledTimerMux = portMUX_INITIALIZER_UNLOCKED;

bool switchReceiverChannel(uint8_t channel) {
  lastChannelAttempt = millis();
  esp_err_t result = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  if (result != ESP_OK) {
    Serial.printf("Failed to switch to WiFi channel %u: %s\n", channel, esp_err_to_name(result));
    return false;
  }
  currentChannel = channel;
  return true;
}

bool setupChannelScan() {
  wifi_country_t country = {};
  esp_err_t result = esp_wifi_get_country(&country);
  if (result != ESP_OK) {
    Serial.printf("Failed to get WiFi country channel range: %s\n", esp_err_to_name(result));
    return false;
  }
  if (country.schan < 1 || country.nchan == 0 ||
      unsigned(country.schan) + country.nchan - 1 > 14) {
    Serial.println("Invalid WiFi country channel range; receiver not started");
    return false;
  }
  firstScanChannel = country.schan;
  lastScanChannel = country.schan + country.nchan - 1;
  uint8_t initialChannel = ESPNOW_START_CHANNEL;
  if (initialChannel < firstScanChannel || initialChannel > lastScanChannel) {
    Serial.println("Preferred starting channel outside country range; using first permitted channel");
    initialChannel = firstScanChannel;
  }
  return switchReceiverChannel(initialChannel);
}

void updateChannelScan() {
  unsigned long now = millis();
  if (channelLocked) {
    if (now - lastValidPacketAt < SIGNAL_LOSS_TIMEOUT) {
      return;
    }
    channelLocked = false;
    Serial.println("Measurement signal lost; resuming channel scan");
  }
  if (now - lastChannelAttempt >= CHANNEL_DWELL_INTERVAL) {
    uint8_t nextChannel = currentChannel >= lastScanChannel ? firstScanChannel : currentChannel + 1;
    if (switchReceiverChannel(nextChannel)) {
      Serial.printf("Scanning WiFi channel %u\n", currentChannel);
    }
  }
}

void IRAM_ATTR onLedTimer() {
  portENTER_CRITICAL_ISR(&ledTimerMux);
  // Ignore a pending old interrupt if a new message has restarted the pulse.
  if (timerRead(ledTimer) >= LED_PULSE_DURATION * 1000ULL) {
    digitalWrite(STATUS_LED_PIN, STATUS_LED_ACTIVE_LOW ? HIGH : LOW);
  }
  portEXIT_CRITICAL_ISR(&ledTimerMux);
}

bool setupLedTimer() {
#if ESP_IDF_VERSION_MAJOR >= 5
  ledTimer = timerBegin(1000000);
#else
  ledTimer = timerBegin(0, 80, true);
#endif
  if (ledTimer == nullptr) {
    Serial.println("Failed to allocate LED hardware timer");
    return false;
  }

  // At 1 MHz, alarm values are microseconds. Arm a one-shot only on reception.
  timerStop(ledTimer);
#if ESP_IDF_VERSION_MAJOR >= 5
  timerAttachInterrupt(ledTimer, &onLedTimer);
#else
  timerAttachInterrupt(ledTimer, &onLedTimer, false);
#endif
  return true;
}

void enqueuePacket(const uint8_t* sender, const uint8_t* data, int length) {
  timerStop(ledTimer);
  timerWrite(ledTimer, 0);
  portENTER_CRITICAL(&ledTimerMux);
  digitalWrite(STATUS_LED_PIN, STATUS_LED_ACTIVE_LOW ? LOW : HIGH);
  portEXIT_CRITICAL(&ledTimerMux);

#if ESP_IDF_VERSION_MAJOR >= 5
  timerAlarm(ledTimer, LED_PULSE_DURATION * 1000ULL, false, 0);
#else
  timerAlarmWrite(ledTimer, LED_PULSE_DURATION * 1000ULL, false);
  timerAlarmEnable(ledTimer);
#endif
  timerStart(ledTimer);

  ReceivedMeasurements received = {};
  memcpy(received.sender, sender, sizeof(received.sender));
  received.length = length;
  received.receivedAt = millis();
  wifi_second_chan_t secondaryChannel;
  received.channelReadResult = esp_wifi_get_channel(&received.channel, &secondaryChannel);
  if (length == int(sizeof(received.packet))) {
    memcpy(&received.packet, data, sizeof(received.packet));
  }

  // The callback runs on the WiFi task; defer validation and Serial output.
  if (xQueueSend(receiveQueue, &received, 0) != pdTRUE) {
    portENTER_CRITICAL(&dropCountMux);
    ++droppedPackets;
    portEXIT_CRITICAL(&dropCountMux);
  }
}

#if ESP_IDF_VERSION_MAJOR >= 5
void onDataReceived(const esp_now_recv_info_t* info, const uint8_t* data, int length) {
  enqueuePacket(info->src_addr, data, length);
}
#else
void onDataReceived(const uint8_t* sender, const uint8_t* data, int length) {
  enqueuePacket(sender, data, length);
}
#endif

void logChannel(unsigned int number, const EspNowChannelMeasurements& channel) {
  Serial.printf("Channel %u: I=%.3fA, P=%.1fW, Injection=%.1fW, S=%.1fVA, "
                "PF=%.2f, E=%.1fWh, Injected E=%.1fWh\n",
                number, channel.current, channel.power, channel.injection,
                channel.apparentPower, channel.powerFactor, channel.energy,
                channel.injectionEnergy);
}

void logPacket(const ReceivedMeasurements& received) {
  Serial.printf("\nFrom %02X:%02X:%02X:%02X:%02X:%02X | bytes=%d\n",
                received.sender[0], received.sender[1], received.sender[2],
                received.sender[3], received.sender[4], received.sender[5],
                received.length);

  if (received.length != int(sizeof(EspNowMeasurements))) {
    Serial.printf("Rejected packet: expected %u bytes\n",
                  static_cast<unsigned int>(sizeof(EspNowMeasurements)));
    return;
  }

  const EspNowMeasurements& packet = received.packet;
  if (packet.magic != ESPNOW_MEASUREMENTS_MAGIC ||
      packet.version != ESPNOW_MEASUREMENTS_VERSION ||
      packet.size != sizeof(EspNowMeasurements)) {
    Serial.printf("Rejected packet: magic=0x%08lX, version=%u, declared size=%u\n",
                  static_cast<unsigned long>(packet.magic), packet.version, packet.size);
    return;
  }

  if (received.channelReadResult != ESP_OK) {
    Serial.printf("Cannot determine received packet's channel: %s\n",
                  esp_err_to_name(received.channelReadResult));
    return;
  }
  if (packet.wifiChannel < 1 || packet.wifiChannel > 14) {
    Serial.printf("Rejected packet: invalid sender WiFi channel %u\n", packet.wifiChannel);
    return;
  }
  Serial.printf("RF channel: sender=%u | receiver at reception=%u | selected=%u\n",
                packet.wifiChannel, received.channel, currentChannel);
  if (packet.wifiChannel != received.channel) {
    Serial.println("Warning: sender and receiver report different WiFi channels");
  }
  // Queued data from a previous channel must not lock or keep alive the new channel.
  if (received.channel != currentChannel ||
      millis() - received.receivedAt >= SIGNAL_LOSS_TIMEOUT) {
    Serial.println("Ignoring stale measurement packet");
    return;
  }
  lastValidPacketAt = received.receivedAt;
  if (!channelLocked) {
    channelLocked = true;
    Serial.printf("Locked onto sender on WiFi channel %u\n", currentChannel);
  }

  Serial.printf("Sequence=%lu | sender uptime=%lums | U=%.2fV\n",
                static_cast<unsigned long>(packet.sequence),
                static_cast<unsigned long>(packet.uptimeMillis), packet.voltage);
  logChannel(1, packet.channels[0]);
  logChannel(2, packet.channels[1]);
}

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, STATUS_LED_ACTIVE_LOW ? HIGH : LOW);
  Serial.begin(115200);
  delay(1000);

  if (!setupLedTimer()) {
    return;
  }
  if (!WiFi.mode(WIFI_STA)) {
    Serial.println("Failed to enable WiFi station mode");
    return;
  }
  if (!setupChannelScan()) {
    return;
  }

  receiveQueue = xQueueCreate(8, sizeof(ReceivedMeasurements));
  if (receiveQueue == nullptr) {
    Serial.println("Failed to allocate ESP-NOW receive queue");
    return;
  }
  esp_err_t result = esp_now_init();
  if (result != ESP_OK) {
    Serial.printf("ESP-NOW initialization failed: %s\n", esp_err_to_name(result));
    return;
  }
  result = esp_now_register_recv_cb(onDataReceived);
  if (result != ESP_OK) {
    Serial.printf("ESP-NOW receive callback registration failed: %s\n", esp_err_to_name(result));
    return;
  }

  receiverReady = true;
  Serial.printf("ESP-NOW receiver scanning channels %u-%u, starting at %u, MAC %s\n",
                firstScanChannel, lastScanChannel, currentChannel, WiFi.macAddress().c_str());
}

void loop() {
  if (!receiverReady) {
    delay(1000);
    return;
  }

  portENTER_CRITICAL(&dropCountMux);
  uint32_t dropped = droppedPackets;
  droppedPackets = 0;
  portEXIT_CRITICAL(&dropCountMux);
  if (dropped > 0) {
    Serial.printf("Receive queue full: dropped %lu packet(s)\n",
                  static_cast<unsigned long>(dropped));
  }

  ReceivedMeasurements received;
  while (xQueueReceive(receiveQueue, &received, 0) == pdTRUE) {
    logPacket(received);
  }
  updateChannelScan();
  delay(10);
}
