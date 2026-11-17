#pragma once

#include <stdint.h>

// Binary protocol v2: little-endian integers and IEEE-754 64-bit doubles.
// Receivers must check packet length, magic and version before using the data.
constexpr uint32_t ESPNOW_MEASUREMENTS_MAGIC = 0x31495855;  // "UXI1"
constexpr uint16_t ESPNOW_MEASUREMENTS_VERSION = 2;

struct EspNowChannelMeasurements {
  double current;          // A RMS
  double power;            // W consumed, nonnegative
  double injection;        // W injected, nonnegative
  double apparentPower;    // VA
  double powerFactor;      // Magnitude, either direction
  double energy;           // Wh consumed
  double injectionEnergy;  // Wh injected
};

struct EspNowMeasurements {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t sequence;        // Starts at zero after reboot
  uint32_t uptimeMillis;    // Time of transmission
  uint8_t wifiChannel;      // Sender's radio channel sampled before sending
  uint8_t reserved[7];      // Zero; explicit padding for the doubles below
  double voltage;          // V RMS, shared by both channels
  EspNowChannelMeasurements channels[2];  // [0]: GPIO32; [1]: GPIO35
};

static_assert(sizeof(double) == 8, "ESP-NOW protocol requires 64-bit doubles");
static_assert(sizeof(EspNowChannelMeasurements) == 56, "Unexpected channel layout");
static_assert(sizeof(EspNowMeasurements) == 144, "Unexpected ESP-NOW packet layout");
static_assert(sizeof(EspNowMeasurements) <= 250, "ESP-NOW v1 payload limit exceeded");
