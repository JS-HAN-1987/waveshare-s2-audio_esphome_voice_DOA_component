#include "esp_sr_doa.h"
#include "esp_timer.h"
#include "esphome/core/log.h"

namespace esphome {
namespace esp_sr_doa {

static const char *TAG = "esp_sr_doa";
static const uint32_t PROCESS_INTERVAL_MS =
    100; // Process more frequently for smoothness

void ESPSRDOA::setup() {
  ESP_LOGI(TAG, "Initializing Shared GCC-PHAT DOA Engine (Custom)...");

  if (!this->doa_engine_.setup()) {
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "GCC-PHAT Engine Initialized.");
}

void ESPSRDOA::loop() {
  // Event driven
}

void ESPSRDOA::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP-SR DOA (Custom GCC-PHAT):");
  LOG_SENSOR("  ", "DOA", this->doa_sensor_);
}

void ESPSRDOA::feed_audio(const std::vector<uint8_t> &data) {
  uint32_t now = esphome::millis();
  float new_angle = 0.0f;

  // Hardcoded fix for ESP32-S3-Box-3 (or similar 4ch hardware)
  // Input `data` is 4-channel interleaved (Ref, Mic1, Ref, Mic2) or similar.
  // We need to extract Ch1 and Ch3 for the DOA engine (Stereo).

  const int16_t *raw_samples = (const int16_t *)data.data();
  size_t total_samples = data.size() / sizeof(int16_t);

  // Create clean stereo buffer
  // Ch1 (Idx 1), Ch3 (Idx 3) from 4-channel frame
  std::vector<int16_t> stereo_samples;
  stereo_samples.reserve(total_samples / 2);

  // Safety check: ensure multiple of 4
  size_t frame_count = total_samples / 4;

  for (size_t i = 0; i < frame_count; i++) {
    stereo_samples.push_back(raw_samples[4 * i + 1]); // Ch1 (Left)
    stereo_samples.push_back(raw_samples[4 * i + 3]); // Ch3 (Right)
  }

  std::vector<uint8_t> stereo_bytes((uint8_t *)stereo_samples.data(),
                                    (uint8_t *)stereo_samples.data() +
                                        stereo_samples.size() *
                                            sizeof(int16_t));

  // Delegate all processing to the shared engine with STEREO data
  if (this->doa_engine_.feed_audio(stereo_bytes, new_angle)) {
    // Only publish if interval passed (optional throttling)
    if (now - this->last_process_time_ >= PROCESS_INTERVAL_MS) {
      this->current_doa_ = new_angle;
      this->last_process_time_ = now;

      if (this->doa_sensor_) {
        this->doa_sensor_->publish_state(this->current_doa_);
      }
    }
  }
}

} // namespace esp_sr_doa
} // namespace esphome
