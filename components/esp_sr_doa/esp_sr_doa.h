#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include <vector>

// ESP-SR DOA Header
#include "esp_afe_doa.h"

namespace esphome {
namespace esp_sr_doa {

class ESPSRDOA : public Component {
public:
  ~ESPSRDOA(); // Destructor for cleanup
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_doa_sensor(sensor::Sensor *sens) { this->doa_sensor_ = sens; }

  float get_doa() const { return this->current_doa_; }

  void feed_audio(const std::vector<uint8_t> &data);

protected:
  sensor::Sensor *doa_sensor_{nullptr};
  afe_doa_handle_t *doa_handle_{nullptr};

  // Fixed-size audio buffer (avoid dynamic allocation)
  // Must match DOA_CHUNK_SIZE to prevent buffer overflow
  static const size_t BUFFER_SIZE = 512;
  int16_t audio_buffer_[BUFFER_SIZE];
  size_t buffer_pos_{0};

  float current_doa_{0.0f};
  uint32_t last_process_time_{0};
  bool is_processing_{false}; // Prevent concurrent processing
};

} // namespace esp_sr_doa
} // namespace esphome
