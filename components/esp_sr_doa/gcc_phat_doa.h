#pragma once

#include "esphome/core/log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstring>
#include <vector>

namespace esphome {
namespace esp_sr_doa {

static const char *DOA_TAG = "GccPhatDoa";

// Constants
static const int FFT_N = 512;
static const float MIC_DISTANCE = 0.045f;
static const int SAMPLE_RATE = 16000;
static const float SOUND_SPEED = 343.0f;

// ---------------------------------------------
// Simple standalone FFT implementation
// Replaces esp-dsp to avoid build/linking issues
// ---------------------------------------------
class SimpleFFT {
public:
  static void fft(std::vector<float> &data) {
    int n = data.size() / 2; // Complex pairs
    if (n < 2)
      return;

    // Bit Reverse
    int j = 0;
    for (int i = 0; i < n; i++) {
      if (j > i) {
        std::swap(data[2 * i], data[2 * j]);
        std::swap(data[2 * i + 1], data[2 * j + 1]);
      }
      int m = n >> 1;
      while (m >= 1 && j >= m) {
        j -= m;
        m >>= 1;
      }
      j += m;
    }

    // Danielson-Lanczos
    for (int mmax = 1; mmax < n; mmax <<= 1) {
      int istep = mmax << 1;
      float theta = -M_PI / mmax;
      float wtemp = sin(0.5f * theta);
      float wpr = -2.0f * wtemp * wtemp;
      float wpi = sin(theta);
      float wr = 1.0f;
      float wi = 0.0f;

      for (int m = 0; m < mmax; m++) {
        for (int i = m; i < n; i += istep) {
          int j = i + mmax;
          float tr = wr * data[2 * j] - wi * data[2 * j + 1];
          float ti =
              wr * data[2 * j + 1] + wi * data[2 * j]; // Fixed sign convention

          data[2 * j] = data[2 * i] - tr;
          data[2 * j + 1] = data[2 * i + 1] - ti;
          data[2 * i] += tr;
          data[2 * i + 1] += ti;
        }
        wtemp = wr;
        wr = wr * wpr - wi * wpi + wr;
        wi = wi * wpr + wtemp * wpi + wi;
      }
    }
  }

  static void ifft(std::vector<float> &data) {
    int n = data.size();
    // Conjugate input
    for (int i = 1; i < n; i += 2) {
      data[i] = -data[i];
    }

    fft(data);

    // Conjugate output and scale
    float inv_N = 1.0f / (float)(n / 2);

    for (int i = 0; i < n; i += 2) {
      data[i] *= inv_N;
      data[i + 1] = -data[i + 1] * inv_N;
    }
  }
};

class GccPhatDoa {
public:
  enum Direction {
    DIR_LEFT = -1,
    DIR_CENTER = 0,
    DIR_RIGHT = 1,
    DIR_UNKNOWN = 99
  };

  GccPhatDoa() {
    this->fft_in_left_.resize(FFT_N * 2);
    this->fft_in_right_.resize(FFT_N * 2);
    this->fft_out_left_.resize(FFT_N * 2);
    this->fft_out_right_.resize(FFT_N * 2);
    this->gcc_accum_.resize(FFT_N * 2);
    // Fill accum with 0
    std::fill(this->gcc_accum_.begin(), this->gcc_accum_.end(), 0.0f);
  }

  bool setup() {
    // No specific initialization needed for SimpleFFT
    return true;
  }

  // Input: Stereo Int16 buffer
  bool feed_audio(const std::vector<uint8_t> &data, float &out_angle) {
    if (data.size() % 4 != 0)
      return false;

    int num_samples = data.size() / sizeof(int16_t);
    int num_frames = num_samples / 2;
    const int16_t *pcm = (const int16_t *)data.data();

    // Use shorter length if data is smaller than FFT
    int copy_len = std::min(num_frames, FFT_N);

    // Clear buffers
    std::fill(this->fft_in_left_.begin(), this->fft_in_left_.end(), 0.0f);
    std::fill(this->fft_in_right_.begin(), this->fft_in_right_.end(), 0.0f);

    float energy_sum = 0.0f;

    for (int i = 0; i < copy_len; i++) {
      float l_val = (float)pcm[2 * i];
      float r_val = (float)pcm[2 * i + 1];

      // Hanning Window
      float window = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (FFT_N - 1)));

      this->fft_in_left_[i * 2] = l_val * window;
      this->fft_in_left_[i * 2 + 1] = 0.0f;

      this->fft_in_right_[i * 2] = r_val * window;
      this->fft_in_right_[i * 2 + 1] = 0.0f;

      energy_sum += fabsf(l_val) + fabsf(r_val);
    }

    // Calibration Logic
    if (!this->calibrated_) {
      // Warmup: Skip first 10 frames (Short warmup)
      static int warmup = 0;
      if (warmup++ < 10)
        return false;

      this->calibration_sum_ += energy_sum / (float)copy_len;
      this->calibration_count_++;
      if (this->calibration_count_ > 3) {
        float avg_noise = this->calibration_sum_ / 3.0f;
        this->noise_threshold_ = avg_noise * 1.5f; // Reduced from 2.5
        // Ensure min threshold
        if (this->noise_threshold_ < 20.0f)
          this->noise_threshold_ = 20.0f; // Lowered clamp

        ESP_LOGI(DOA_TAG,
                 "Calibration Complete. Noise Floor: %.1f, Threshold: %.1f",
                 avg_noise, this->noise_threshold_);
        this->calibrated_ = true;
      }
      return false;
    }

    float avg_energy = energy_sum / (float)copy_len;

    // Debug logging periodically
    static int log_cnt = 0;
    if (++log_cnt % 100 == 0) {
      ESP_LOGI(DOA_TAG, "Energy: %.1f, Thresh: %.1f, Calibrated: %d",
               avg_energy, this->noise_threshold_, this->calibrated_);
    }

    if (avg_energy < this->noise_threshold_) {
      return false;
    }

    float res = this->process_doa_();
    if (std::isnan(res))
      return false;

    out_angle = res;
    return true;
  }

private:
  std::vector<float> fft_in_left_;
  std::vector<float> fft_in_right_;
  std::vector<float> fft_out_left_;
  std::vector<float> fft_out_right_;

  std::vector<float> gcc_accum_;
  int accum_count_ = 0;
  static const int ACCUM_FRAMES = 2; // User requested 2 frames (Hyper fast)

  bool calibrated_ = false;
  float calibration_sum_ = 0.0f;
  int calibration_count_ = 0;
  float noise_threshold_ = 20.0f; // Lower default min threshold

  float last_output_angle_ = 0.0f;

  float process_doa_() {
    this->fft_out_left_ = this->fft_in_left_;
    this->fft_out_right_ = this->fft_in_right_;

    SimpleFFT::fft(this->fft_out_left_);
    SimpleFFT::fft(this->fft_out_right_);

    // Freq Masking (200Hz - 3.5kHz)
    int bin_min = 6;
    int bin_max = 112;

    for (int i = 0; i < FFT_N; i++) {
      bool keep = false;
      if ((i >= bin_min && i <= bin_max) ||
          (i >= (FFT_N - bin_max) && i <= (FFT_N - bin_min))) {
        keep = true;
      }

      if (!keep) {
        this->fft_out_left_[i * 2] = 0;
        this->fft_out_left_[i * 2 + 1] = 0;
        this->fft_out_right_[i * 2] = 0;
        this->fft_out_right_[i * 2 + 1] = 0;
      }
    }

    // GCC-PHAT
    for (int i = 0; i < FFT_N; i++) {
      float lr = this->fft_out_left_[i * 2];
      float li = this->fft_out_left_[i * 2 + 1];
      float rr = this->fft_out_right_[i * 2];
      float ri = this->fft_out_right_[i * 2 + 1];

      // Complex Mul: L * Conj(R)
      float xr = lr * rr + li * ri; // Real
      float xi = li * rr - lr * ri; // Imag

      float mag = sqrtf(xr * xr + xi * xi);
      if (mag > 1e-9f) {
        xr /= mag;
        xi /= mag;
      }

      this->fft_out_left_[i * 2] = xr;
      this->fft_out_left_[i * 2 + 1] = xi;
    }

    SimpleFFT::ifft(this->fft_out_left_);

    // Accumulate Real part
    for (int i = 0; i < FFT_N; i++) {
      // Real part is at 2*i. Note: gcc_accum_ is size FFT_N * 2 but we only
      // strictly necessarily need size FFT_N for real GCC. But for simplicity
      // of indexing let's just use the same size and access 2*i? Actually,
      // let's just accumulate into linear indices to save space/logic if we
      // wanted, but let's stick to the 2*i indexing to match the buffer
      // structure if that's what we allocated. Wait, I allocated FFT_N * 2
      this->gcc_accum_[i * 2] += this->fft_out_left_[i * 2];
    }
    this->accum_count_++;

    if (this->accum_count_ < ACCUM_FRAMES) {
      return NAN;
    }

    // Peak Finding
    int search_wnd = 10;
    float max_val = -1e9f;
    int best_lag = 0;

    // Helper lambda for wrapping index
    auto get_lag_val = [&](int lag) -> float {
      int idx;
      if (lag < 0)
        idx = FFT_N + lag;
      else
        idx = lag;
      return this->gcc_accum_[idx * 2];
    };

    for (int k = -search_wnd; k <= search_wnd; k++) {
      float val = get_lag_val(k);
      if (val > max_val) {
        max_val = val;
        best_lag = k;
      }
    }

    // Quadratic Interpolation
    float val_prev = get_lag_val(best_lag - 1);
    float val_curr = max_val;
    float val_next = get_lag_val(best_lag + 1);

    float denom = 2.0f * (val_prev - 2.0f * val_curr + val_next);
    float delta = 0.0f;
    if (std::abs(denom) > 1e-6f) {
      delta = (val_prev - val_next) / denom;
    }

    float true_lag = (float)best_lag + delta;

    // Clear accumulator
    std::fill(this->gcc_accum_.begin(), this->gcc_accum_.end(), 0.0f);
    this->accum_count_ = 0;

    // Lag to Angle
    // Fs = 16000. True Lag is in samples.
    // Time delay = true_lag / Fs
    // Sin(theta) = Time delay * Speed / MicDist
    float tau = true_lag / (float)SAMPLE_RATE;
    float sin_val = (tau * SOUND_SPEED) / MIC_DISTANCE;

    if (sin_val > 1.0f)
      sin_val = 1.0f;
    if (sin_val < -1.0f)
      sin_val = -1.0f;

    float angle_rad = asinf(sin_val);
    float angle_deg = angle_rad * 180.0f / (float)M_PI;

    // EMA Smoothing
    float alpha = 0.6f;
    this->last_output_angle_ =
        (alpha * angle_deg) + ((1.0f - alpha) * this->last_output_angle_);

    return this->last_output_angle_;
  }
};

} // namespace esp_sr_doa
} // namespace esphome
