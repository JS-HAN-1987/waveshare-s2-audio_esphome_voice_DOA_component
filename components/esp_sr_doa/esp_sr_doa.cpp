#include "esp_sr_doa.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <cstring>

namespace esphome {
namespace esp_sr_doa {

static const char *TAG = "esp_sr_doa";

static const int SAMPLE_RATE = 16000;
static const float DOA_RESOLUTION = 10.0f;
static const float MIC_DISTANCE = 0.06f;
static const size_t DOA_CHUNK_SIZE = 512;
// Increased interval to reduce CPU load and prevent watchdog timeout
static const uint32_t PROCESS_INTERVAL_MS = 5000;

void ESPSRDOA::setup() {
  ESP_LOGI(TAG, "Initializing ESP-SR DOA Component...");

  // Initialize buffer
  memset(this->audio_buffer_, 0, sizeof(this->audio_buffer_));
  this->buffer_pos_ = 0;

  this->doa_handle_ = afe_doa_create("MM", SAMPLE_RATE, DOA_RESOLUTION,
                                     MIC_DISTANCE, DOA_CHUNK_SIZE);

  if (!this->doa_handle_) {
    ESP_LOGE(TAG, "Failed to create DOA handle!");
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "ESP-SR DOA Initialized.");
}

void ESPSRDOA::loop() {}

void ESPSRDOA::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP-SR DOA:");
  LOG_SENSOR("  ", "DOA", this->doa_sensor_);
}

void ESPSRDOA::feed_audio(const std::vector<uint8_t> &data) {
  // Early return checks
  if (!this->doa_handle_ || data.empty()) {
    return;
  }

  // Prevent concurrent processing (thread safety)
  if (this->is_processing_) {
    ESP_LOGV(TAG, "Already processing, skipping this batch");
    return;
  }

  // If buffer is full, discard incoming data
  if (this->buffer_pos_ >= BUFFER_SIZE) {
    ESP_LOGV(TAG, "Buffer full, discarding incoming data");
    return;
  }

  // Copy data to fixed buffer
  const int16_t *samples = reinterpret_cast<const int16_t *>(data.data());
  size_t num_samples = data.size() / sizeof(int16_t);

  // Copy only what fits
  size_t space_available = BUFFER_SIZE - this->buffer_pos_;
  size_t samples_to_copy =
      (num_samples < space_available) ? num_samples : space_available;

  for (size_t i = 0; i < samples_to_copy; i++) {
    this->audio_buffer_[this->buffer_pos_++] = samples[i];
  }

  // Check if we have enough data AND enough time has passed
  uint32_t now = millis();
  if (this->buffer_pos_ < BUFFER_SIZE) {
    return; // Not enough data yet
  }

  if (now - this->last_process_time_ < PROCESS_INTERVAL_MS) {
    // Reset buffer even if we don't process, to avoid overflow
    this->buffer_pos_ = 0;
    return; // Too soon to process again
  }

  // Mark as processing
  this->is_processing_ = true;
  this->last_process_time_ = now;

  // Process DOA with error handling
  float doa_result = 0.0f;

  // Yield to watchdog before heavy processing
  yield();

  // Call the ESP-SR DOA processing function
  doa_result = afe_doa_process(this->doa_handle_, this->audio_buffer_);

  // Yield again after processing
  yield();

  // Validate result (DOA should be between -180 and 180 degrees)
  if (doa_result >= -180.0f && doa_result <= 180.0f) {
    this->current_doa_ = doa_result;

    // Publish
    if (this->doa_sensor_) {
      this->doa_sensor_->publish_state(this->current_doa_);
      ESP_LOGI(TAG, "DOA: %.1f degrees", this->current_doa_);
    }
  } else {
    ESP_LOGW(TAG, "Invalid DOA result: %.1f, discarding", doa_result);
  }

  // Reset buffer position
  this->buffer_pos_ = 0;

  // Clear processing flag
  this->is_processing_ = false;
}

} // namespace esp_sr_doa
} // namespace esphome
