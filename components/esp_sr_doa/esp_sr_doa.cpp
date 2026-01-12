#include "esp_sr_doa.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// ESP32-specific headers for crashdiagnostics
#ifdef USE_ESP32
#include "esp_err.h" // Needed for esp_err_t, ESP_OK
#include "esp_system.h"
#include "sdkconfig.h" // Needed for CONFIG_DSP_MAX_FFT_SIZE

// Note: Do NOT include esp_log.h - it conflicts with ESPHome's logging
// ESPHome's ESP_LOG* macros from esphome/core/log.h are used instead
#endif

// ESP-DSP headers
#include "dsps_fft2r.h"
#include "dsps_wind.h"
#include "esp_dsp.h"

namespace esphome {
namespace esp_sr_doa {

static const char *TAG = "esp_sr_doa";

// Parameters
static const int SAMPLE_RATE = 16000;
static const float MIC_DISTANCE = 0.06f;         // 6cm
static const float SPEED_OF_SOUND = 343.0f;      // m/s
static const uint32_t PROCESS_INTERVAL_MS = 200; // Update 5 times per second

void ESPSRDOA::setup() {
  ESP_LOGI(TAG, "Initializing Light-weight GCC-PHAT DOA...");

  // Initialize FFT
  esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Not possible to initialize FFT. Error = %d", ret);
    this->mark_failed();
    return;
  }

  // Initialize Window
  this->init_window_();

  // Initialize buffers
  memset(this->mic_buffer_, 0, sizeof(this->mic_buffer_));
  this->buffer_pos_ = 0;

  ESP_LOGI(TAG, "GCC-PHAT DOA Initialized. FFT Size: %d", FFT_N);
}

void ESPSRDOA::init_window_() {
  // Generate Hann window
  dsps_wind_hann_f32(this->window_, FFT_N);
}

void ESPSRDOA::loop() {}

void ESPSRDOA::dump_config() {
  ESP_LOGCONFIG(TAG, "Light-weight GCC-PHAT DOA:");
  ESP_LOGCONFIG(TAG, "  FFT Size: %d", FFT_N);
  ESP_LOGCONFIG(TAG, "  Mic Distance: %.2fm", MIC_DISTANCE);
  LOG_SENSOR("  ", "DOA", this->doa_sensor_);
}

void ESPSRDOA::feed_audio(const std::vector<uint8_t> &data) {
  if (data.empty())
    return;

  // Cast to int16 samples
  const int16_t *samples = reinterpret_cast<const int16_t *>(data.data());
  size_t num_samples = data.size() / sizeof(int16_t);

  // Fill internal buffer
  // We assume input is stereo interleaved (L, R, L, R...)
  // We need to fill FFT_N samples (which is FFT_N * 2 int16s due to stereo)

  for (size_t i = 0; i < num_samples; i++) {
    if (this->buffer_pos_ < FFT_N * 2) {
      this->mic_buffer_[this->buffer_pos_++] = samples[i];
    } else {
      // Buffer full, time to process?
      break;
    }
  }

  // Check if buffer is full and enough time passed
  uint32_t now = millis();
  if (this->buffer_pos_ >= FFT_N * 2) {
    if (now - this->last_process_time_ >= PROCESS_INTERVAL_MS &&
        !this->is_processing_) {
      this->is_processing_ = true;
      this->process_doa_();
      this->last_process_time_ = now;
      this->is_processing_ = false;
    }
    // Reset buffer pos (simple overlap-save not implemented for simplicity,
    // just drop)
    this->buffer_pos_ = 0;
  }
}

void ESPSRDOA::process_doa_() {
  // Separate channels and apply window
  for (int i = 0; i < FFT_N; i++) {
    // Left channel (even indices)
    // Convert to float, apply window
    // FFT input expects Real/Imag interleaved. Imag is 0.
    this->fft_input_left_[i * 2 + 0] =
        (float)this->mic_buffer_[i * 2] * this->window_[i];
    this->fft_input_left_[i * 2 + 1] = 0.0f;

    // Right channel (odd indices)
    this->fft_input_right_[i * 2 + 0] =
        (float)this->mic_buffer_[i * 2 + 1] * this->window_[i];
    this->fft_input_right_[i * 2 + 1] = 0.0f;
  }

  // Perform FFT
  // Bit reverse happens internally in dsps_fft2r_fc32 if configured?
  // Normally dsps_fft2r_fc32 does the FFT.
  // We copy to out buffers first because FFT is in-place
  std::copy(std::begin(this->fft_input_left_), std::end(this->fft_input_left_),
            std::begin(this->fft_out_left_));
  std::copy(std::begin(this->fft_input_right_),
            std::end(this->fft_input_right_), std::begin(this->fft_out_right_));

  dsps_fft2r_fc32(this->fft_out_left_, FFT_N);
  dsps_bit_rev_fc32(this->fft_out_left_, FFT_N);

  dsps_fft2r_fc32(this->fft_out_right_, FFT_N);
  dsps_bit_rev_fc32(this->fft_out_right_, FFT_N);

  // Compute Generalized Cross Correlation (GCC-PHAT) in Frequency Domain
  // R = X1 * conj(X2) / |X1 * conj(X2)|
  for (int i = 0; i < FFT_N; i++) {
    float real1 = this->fft_out_left_[i * 2];
    float imag1 = this->fft_out_left_[i * 2 + 1];
    float real2 = this->fft_out_right_[i * 2];
    float imag2 = this->fft_out_right_[i * 2 + 1];

    // Complex Conjugate of X2: (real2, -imag2)
    // Mult: (r1*r2 - i1*(-i2), r1*(-i2) + i1*r2)
    float product_real = real1 * real2 + imag1 * imag2;
    float product_imag = imag1 * real2 - real1 * imag2;

    // Magnitude
    float mag =
        sqrtf(product_real * product_real + product_imag * product_imag) +
        1e-6f; // Avoid div by zero

    // Normalize (PHAT weighting)
    // Store in fft_out_left_ for IFFT (we reuse buffer)
    this->fft_out_left_[i * 2] = product_real / mag;
    this->fft_out_left_[i * 2 + 1] = product_imag / mag;
  }

  // Inverse FFT to get Time Domain Correlation
  // dsps_fft2r_fc32 is Forward FFT. For Inverse:
  // Swap Real/Imag parts -> FFT -> Swap back & Scale
  // Or simply define IFFT using standard trick: conj(FFT(conj(X))) / N

  // Conjugate input
  for (int i = 0; i < FFT_N; i++) {
    this->fft_out_left_[i * 2 + 1] = -this->fft_out_left_[i * 2 + 1];
  }

  dsps_fft2r_fc32(this->fft_out_left_, FFT_N);
  dsps_bit_rev_fc32(this->fft_out_left_, FFT_N);

  // Conjugate output (and scale by N, but finding max doesn't care about scale)
  // And map to correlation buffer with circular shift handling
  // Expected lag is around 0. Lag 0 is at index 0.
  // Positive lags [0..N/2], Negative lags [N/2..N-1]

  // Copy to linear buffer centered at N/2 for easier peak finding
  // xcorr index: 0..N-1
  // Layout: [N/2..N-1] (Negative) followed by [0..N/2] (Positive) - No wait.
  // FFT result: [0] = lag 0, [1] = lag 1, [N-1] = lag -1

  // Find Peak
  float max_val = -1.0f;
  int max_idx = 0;

  // Search range limited by mic distance
  // Max delay samples = Distance / Speed * Rate
  // 0.06m / 343m/s * 16000Hz ~= 2.8 samples.
  // Wait, 2.8 samples is VERY small. This resolution is too low for 16kHz & 6cm
  // spacing? Yes, for 6cm, max lag is ~3 samples. To get better resolution,
  // usually we upscale (interpolation) or use higher sample rate. However,
  // simple GCC-PHAT gives coarse direction. Let's check search range: +/- 4
  // samples.

  int search_radius = 6; // slightly more than theoretical max 3

  // Check positive lags [0..search_radius]
  for (int i = 0; i <= search_radius; i++) {
    float mag =
        sqrtf(this->fft_out_left_[i * 2] * this->fft_out_left_[i * 2] +
              this->fft_out_left_[i * 2 + 1] * this->fft_out_left_[i * 2 + 1]);
    if (mag > max_val) {
      max_val = mag;
      max_idx = i;
    }
  }

  // Check negative lags [N-search_radius .. N-1]
  for (int i = 1; i <= search_radius; i++) {
    int idx = FFT_N - i;
    float mag = sqrtf(
        this->fft_out_left_[idx * 2] * this->fft_out_left_[idx * 2] +
        this->fft_out_left_[idx * 2 + 1] * this->fft_out_left_[idx * 2 + 1]);
    if (mag > max_val) {
      max_val = mag;
      max_idx = -i;
    }
  }

  // Convert Lag to Angle
  // Lag is in samples (tau)
  // tau = d * sin(theta) / c
  // theta = asin(tau * c / (d * Fs))

  float tau = (float)max_idx;
  // Simple parabolic interpolation could improve sub-sample accuracy here

  float arg = (tau * SPEED_OF_SOUND) / (MIC_DISTANCE * SAMPLE_RATE);

  // Clamp argument for asin
  if (arg > 1.0f)
    arg = 1.0f;
  if (arg < -1.0f)
    arg = -1.0f;

  float theta_rad = asinf(arg);
  float theta_deg = theta_rad * 180.0f / M_PI;

  // Map +90 to -90 to 0-180 or similar based on sensor orientation?
  // Assuming 0 is front. + is right, - is left.
  // Let's publish as is (+/- 90 degrees) for now, or map to 360 circle if
  // needed. Typical usage: 0 is center.

  // Filter/Smoothing could be added here

  if (this->doa_sensor_) {
    this->doa_sensor_->publish_state(theta_deg);
    ESP_LOGD(TAG, "DOA: %.1f deg (Lag: %d)", theta_deg, max_idx);
  }
}

} // namespace esp_sr_doa
} // namespace esphome
