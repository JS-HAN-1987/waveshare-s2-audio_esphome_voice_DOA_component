#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include <complex>
#include <vector>

namespace esphome {
namespace esp_sr_doa {

// FFT configuration
static const int FFT_N = 512; // Must be power of 2

class ESPSRDOA : public Component {
public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_doa_sensor(sensor::Sensor *sens) { this->doa_sensor_ = sens; }

  float get_doa() const { return this->current_doa_; }

  void feed_audio(const std::vector<uint8_t> &data);

protected:
  sensor::Sensor *doa_sensor_{nullptr};

  // Audio Processing Buffers
  // We use simpler buffering for GCC-PHAT
  int16_t mic_buffer_[FFT_N * 2]; // Interleaved stereo samples
  size_t buffer_pos_{0};

  // FFT Buffers
  float fft_input_left_[FFT_N * 2];  // Complex: Real, Imag interleaved
  float fft_input_right_[FFT_N * 2]; // Complex: Real, Imag interleaved
  float fft_out_left_[FFT_N * 2];
  float fft_out_right_[FFT_N * 2];
  float window_[FFT_N];

  // Working buffer for cross-correlation
  float xcorr_[FFT_N * 2];

  float current_doa_{0.0f};
  uint32_t last_process_time_{0};
  bool is_processing_{false};

  // Internal logic
  void process_doa_();
  void init_window_();
};

} // namespace esp_sr_doa
} // namespace esphome
