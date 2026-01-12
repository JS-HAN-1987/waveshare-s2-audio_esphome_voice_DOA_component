#include "esp_sr_doa.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <cstring>

// ESP32-specific headers for crash diagnostics
#ifdef USE_ESP32
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"

#endif

namespace esphome {
namespace esp_sr_doa {

static const char *TAG = "esp_sr_doa";

static const int SAMPLE_RATE = 16000;
static const float DOA_RESOLUTION = 10.0f;
static const float MIC_DISTANCE = 0.06f;
static const size_t DOA_CHUNK_SIZE = 512;
// Increased interval to reduce CPU load and prevent watchdog timeout
static const uint32_t PROCESS_INTERVAL_MS = 5000;

#ifdef USE_ESP32
// Helper function to get reset reason as string
const char *get_reset_reason_str(esp_reset_reason_t reason) {
  switch (reason) {
  case ESP_RST_UNKNOWN:
    return "UNKNOWN - Reset reason cannot be determined";
  case ESP_RST_POWERON:
    return "POWERON - Reset due to power-on event";
  case ESP_RST_EXT:
    return "EXTERNAL - Reset by external pin";
  case ESP_RST_SW:
    return "SOFTWARE - Software reset via esp_restart";
  case ESP_RST_PANIC:
    return "PANIC - Software reset due to exception/panic";
  case ESP_RST_INT_WDT:
    return "INT_WDT - Reset (software or hardware) due to interrupt watchdog";
  case ESP_RST_TASK_WDT:
    return "TASK_WDT - Reset due to task watchdog";
  case ESP_RST_WDT:
    return "WDT - Reset due to other watchdogs";
  case ESP_RST_DEEPSLEEP:
    return "DEEPSLEEP - Reset after exiting deep sleep mode";
  case ESP_RST_BROWNOUT:
    return "BROWNOUT - Brownout reset (software or hardware)";
  case ESP_RST_SDIO:
    return "SDIO - Reset over SDIO";
  default:
    return "UNDEFINED";
  }
}
#endif

void ESPSRDOA::setup() {
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "ESP-SR DOA Component v2.1.0");
  ESP_LOGI(TAG, "Build Date: %s %s", __DATE__, __TIME__);
  ESP_LOGI(
      TAG,
      "Features: Crash diagnostics, Memory monitoring, Reset reason logging");
  ESP_LOGI(TAG, "========================================");

#ifdef USE_ESP32
  // Log reset reason for crash diagnostics
  esp_reset_reason_t reason = esp_reset_reason();
  ESP_LOGI(TAG, "ESP32 Reset Reason: %s", get_reset_reason_str(reason));

  // If the last reset was due to panic or watchdog, log it prominently
  if (reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT ||
      reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT) {
    ESP_LOGE(TAG, "!!! WARNING: Previous crash detected !!!");
    ESP_LOGE(TAG, "!!! Reason: %s !!!", get_reset_reason_str(reason));
    ESP_LOGE(TAG, "!!! Check logs above for stack trace !!!");
  }
#endif

  ESP_LOGI(TAG, "Initializing buffer (size: %d samples)", BUFFER_SIZE);
  memset(this->audio_buffer_, 0, sizeof(this->audio_buffer_));
  this->buffer_pos_ = 0;

  ESP_LOGI(TAG, "Creating DOA handle with parameters:");
  ESP_LOGI(TAG, "  - Sample Rate: %d Hz", SAMPLE_RATE);
  ESP_LOGI(TAG, "  - DOA Resolution: %.1f degrees", DOA_RESOLUTION);
  ESP_LOGI(TAG, "  - Mic Distance: %.2f meters", MIC_DISTANCE);
  ESP_LOGI(TAG, "  - Chunk Size: %d samples", DOA_CHUNK_SIZE);

  this->doa_handle_ = afe_doa_create("MM", SAMPLE_RATE, DOA_RESOLUTION,
                                     MIC_DISTANCE, DOA_CHUNK_SIZE);

  if (!this->doa_handle_) {
    ESP_LOGE(TAG, "========================================");
    ESP_LOGE(TAG, "!!! FATAL ERROR: Failed to create DOA handle !!!");
    ESP_LOGE(TAG, "Possible causes:");
    ESP_LOGE(TAG, "  1. Insufficient memory (check heap)");
    ESP_LOGE(TAG, "  2. Invalid DOA parameters");
    ESP_LOGE(TAG, "  3. ESP-SR library not properly linked");
    ESP_LOGE(TAG, "========================================");
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "DOA handle created successfully");
  ESP_LOGI(TAG, "ESP-SR DOA Component initialized successfully");
  ESP_LOGI(TAG, "Processing interval: %d ms", PROCESS_INTERVAL_MS);
  ESP_LOGI(TAG, "========================================");
}

void ESPSRDOA::loop() {}

void ESPSRDOA::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP-SR DOA:");
  LOG_SENSOR("  ", "DOA", this->doa_sensor_);
}

void ESPSRDOA::feed_audio(const std::vector<uint8_t> &data) {
  // Early return checks
  if (!this->doa_handle_) {
    ESP_LOGW(TAG, "feed_audio called but DOA handle is null!");
    return;
  }

  if (data.empty()) {
    ESP_LOGV(TAG, "feed_audio called with empty data");
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

#ifdef USE_ESP32
  // Log available heap before DOA processing
  uint32_t free_heap_before = esp_get_free_heap_size();
  ESP_LOGV(TAG, "Starting DOA processing - Free heap: %d bytes",
           free_heap_before);
#endif

  // Process DOA with error handling
  float doa_result = 0.0f;

  // Yield to watchdog before heavy processing
  yield();

  ESP_LOGV(TAG, "Calling afe_doa_process with %d samples...", BUFFER_SIZE);

  // Call the ESP-SR DOA processing function
  // This is where crashes might occur if there are issues
  doa_result = afe_doa_process(this->doa_handle_, this->audio_buffer_);

  // Yield again after processing
  yield();

#ifdef USE_ESP32
  // Log heap after processing to detect memory leaks
  uint32_t free_heap_after = esp_get_free_heap_size();
  int32_t heap_diff = free_heap_after - free_heap_before;

  if (heap_diff < 0) {
    ESP_LOGW(TAG, "Memory potentially leaked: %d bytes", -heap_diff);
  }

  ESP_LOGV(TAG, "DOA processing complete - Free heap: %d bytes (diff: %d)",
           free_heap_after, heap_diff);

  // Warn if heap is getting low
  if (free_heap_after < 10000) {
    ESP_LOGW(TAG, "!!! LOW HEAP WARNING: Only %d bytes free !!!",
             free_heap_after);
  }
#endif

  ESP_LOGV(TAG, "Raw DOA result: %.1f degrees", doa_result);

  // Validate result (DOA should be between -180 and 180 degrees)
  if (doa_result >= -180.0f && doa_result <= 180.0f) {
    this->current_doa_ = doa_result;

    // Publish
    if (this->doa_sensor_) {
      this->doa_sensor_->publish_state(this->current_doa_);
      ESP_LOGI(TAG, "DOA: %.1f degrees", this->current_doa_);
    }
  } else {
    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, "Invalid DOA result: %.1f degrees", doa_result);
    ESP_LOGW(TAG, "Expected range: -180.0 to 180.0");
    ESP_LOGW(TAG, "Possible causes:");
    ESP_LOGW(TAG, "  1. Insufficient audio data");
    ESP_LOGW(TAG, "  2. Corrupted audio buffer");
    ESP_LOGW(TAG, "  3. DOA algorithm error");
    ESP_LOGW(TAG, "Discarding this result");
    ESP_LOGW(TAG, "========================================");
  }

  // Reset buffer position
  this->buffer_pos_ = 0;

  // Clear processing flag
  this->is_processing_ = false;

  ESP_LOGV(TAG, "feed_audio completed successfully");
}

} // namespace esp_sr_doa
} // namespace esphome
