#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include <cmath>
#include <cstring>
#include <vector>

#if __has_include("dsps_fft2r.h")
#include "dsps_fft2r.h"
#include "dsps_wind_hann.h"
#else
#endif

// Constants
static const char *DOA_TAG = "GccPhatDoa";
static const int FFT_N = 512;
static const int SAMPLE_RATE = 16000;
static const float MIC_DISTANCE = 0.040f;
static const float SPEED_OF_SOUND = 343.0f;

// ========================================================================
// 개선된 DOA 알고리즘: 좌/우 방향 투표 시스템
// ========================================================================
// 목표:
// 1. 정확한 좌/우 방향 구분 (정밀한 각도보다 방향이 중요)
// 2. 안정적인 출력 (노이즈에 강함)
// 3. 1Hz 정도의 출력 빈도
// ========================================================================

// 방향 열거형
enum class Direction { LEFT, CENTER, RIGHT, UNKNOWN };

class GccPhatDoa {
public:
  GccPhatDoa() = default;
  ~GccPhatDoa() = default;

  bool setup() {
#ifdef CONFIG_DSP_MAX_FFT_SIZE
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
#else
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, FFT_N);
#endif

    if (ret != ESP_OK)
      return false;

    dsps_wind_hann_f32(this->window_, FFT_N);
    memset(this->mic_buffer_, 0, sizeof(this->mic_buffer_));
    this->buffer_pos_ = 0;
    this->last_output_angle_ = 0.0f;
    this->last_output_time_ms_ = 0;
    return true;
  }

  // 메인 인터페이스: 오디오 데이터를 받아서 각도를 출력
  // 반환값: true면 새로운 각도 값이 있음
  bool feed_audio(const std::vector<uint8_t> &data, float &out_angle) {
    if (data.empty())
      return false;
    const int16_t *samples = reinterpret_cast<const int16_t *>(data.data());
    size_t num_samples = data.size() / sizeof(int16_t);
    bool updated = false;

    for (size_t i = 0; i < num_samples; i++) {
      // 버퍼에 샘플 저장
      this->mic_buffer_[this->buffer_pos_++] = samples[i];

      // 버퍼가 가득 차면 처리
      if (this->buffer_pos_ >= FFT_N * 2) {
        float res = this->process_doa_();
        this->buffer_pos_ = 0;
        if (!std::isnan(res)) {
          out_angle = res;
          updated = true;
        }
      }
    }
    return updated;
  }

  // 방향 문자열 얻기 (편의 함수)
  static const char *get_direction_str(float angle) {
    if (angle < -15.0f)
      return "LEFT";
    else if (angle > 15.0f)
      return "RIGHT";
    else
      return "CENTER";
  }

private:
  int16_t mic_buffer_[FFT_N * 2];
  size_t buffer_pos_{0};
  float fft_input_left_[FFT_N * 2];
  float fft_input_right_[FFT_N * 2];
  float fft_out_left_[FFT_N * 2];
  float window_[FFT_N];

  float last_output_angle_{0.0f};
  uint32_t last_output_time_ms_{0};

  // 노이즈 캘리브레이션
  float noise_threshold_{50.0f};
  float calib_energy_left_{0.0f};
  float calib_energy_right_{0.0f};
  bool is_calibrated_{false};
  int calibration_count_{0};
  float calibration_sum_left_{0.0f};
  float calibration_sum_right_{0.0f};
  static const int CALIBRATION_FRAMES = 30;

  // 투표 시스템: 방향별 투표 수
  int vote_left_{0};
  int vote_right_{0};
  int vote_center_{0};
  int total_votes_{0};

  // 최소 출력 간격 (밀리초) - 약 1Hz
  static const uint32_t MIN_OUTPUT_INTERVAL_MS = 800;
  // 결정에 필요한 최소 투표 수
  static const int MIN_VOTES_FOR_DECISION = 10; // ~500ms 분량

  float process_doa_() {
    // ===== 1단계: 양 채널 에너지 계산 (절대값 합) =====
    float energy_left = 0.0f;
    float energy_right = 0.0f;
    for (int i = 0; i < FFT_N; i++) {
      energy_left += std::abs(this->mic_buffer_[i * 2]);
      energy_right += std::abs(this->mic_buffer_[i * 2 + 1]);
    }
    energy_left /= FFT_N;
    energy_right /= FFT_N;

    // ===== 2단계: 노이즈 캘리브레이션 =====
    if (!this->is_calibrated_) {
      this->calibration_sum_left_ += energy_left;
      this->calibration_sum_right_ += energy_right;
      this->calibration_count_++;

      if (this->calibration_count_ >= CALIBRATION_FRAMES) {
        this->calib_energy_left_ =
            this->calibration_sum_left_ / this->calibration_count_;
        this->calib_energy_right_ =
            this->calibration_sum_right_ / this->calibration_count_;

        // 노이즈 바닥의 2배 + 약간의 마진 for VAD threshold
        float avg_noise =
            (this->calib_energy_left_ + this->calib_energy_right_) / 2.0f;
        this->noise_threshold_ = avg_noise * 1.5f + 10.0f;

        ESP_LOGI(
            DOA_TAG,
            "Calibration Complete. Left: %.1f, Right: %.1f, Threshold: %.1f",
            this->calib_energy_left_, this->calib_energy_right_,
            this->noise_threshold_);
        this->is_calibrated_ = true;
      }
      return NAN;
    }

    // ===== 3단계: 음성 활성화 감지 (VAD) =====
    // 단순히 에너지 레벨로만 체크
    bool is_voice_active = (energy_left > this->noise_threshold_) ||
                           (energy_right > this->noise_threshold_);

    if (!is_voice_active) {
      // 음성이 없으면 투표하지 않음
      return NAN;
    }

    // ===== 4단계: GCC-PHAT로 시간차 계산 (다시 활성화) =====
    // De-interleave + Window
    for (int i = 0; i < FFT_N; i++) {
      this->fft_input_left_[i * 2 + 0] =
          (float)this->mic_buffer_[i * 2] * this->window_[i];
      this->fft_input_left_[i * 2 + 1] = 0;
      this->fft_input_right_[i * 2 + 0] =
          (float)this->mic_buffer_[i * 2 + 1] * this->window_[i];
      this->fft_input_right_[i * 2 + 1] = 0;
    }

    // FFT
    dsps_fft2r_fc32(this->fft_input_left_, FFT_N);
    dsps_fft2r_fc32(this->fft_input_right_, FFT_N);
    dsps_bit_rev_fc32(this->fft_input_left_, FFT_N);
    dsps_bit_rev_fc32(this->fft_input_right_, FFT_N);

    // GCC-PHAT: Cross-correlation in frequency domain
    for (int i = 0; i < FFT_N; i++) {
      float r1 = this->fft_input_left_[i * 2];
      float i1 = this->fft_input_left_[i * 2 + 1];
      float r2 = this->fft_input_right_[i * 2];
      float i2 = this->fft_input_right_[i * 2 + 1];

      // X1 * conj(X2)
      float pr = r1 * r2 + i1 * i2;
      float pi = i1 * r2 - r1 * i2;
      float mag = sqrtf(pr * pr + pi * pi) + 1e-6f;

      this->fft_out_left_[i * 2] = pr / mag;
      this->fft_out_left_[i * 2 + 1] = pi / mag;
    }

    // ===== Frequency Masking (Band-pass) =====
    // 4cm 간격 마이크의 앨리어싱 한계(Spatial Aliasing)는 약 4.2kHz입니다.
    // 또한 음성 대역 밖의 노이즈가 시간차 계산을 방해할 수 있습니다.
    // 200Hz ~ 3500Hz 대역만 사용하여 안정성을 높입니다.
    // Resolution: 16000 / 512 = 31.25 Hz/bin
    // Min Bin: 200 / 31.25 = ~6
    // Max Bin: 3500 / 31.25 = ~112

    // DC ~ 200Hz 삭제
    for (int i = 0; i < 6; i++) {
      this->fft_out_left_[i * 2] = 0;
      this->fft_out_left_[i * 2 + 1] = 0;
    }
    // 3500Hz ~ Nyquist 삭제
    for (int i = 112; i < FFT_N / 2;
         i++) { // 절반까지만 유효한 데이터 (Real FFT 고려 시)
      // dsps_fft2r_fc32는 Complex FFT이므로 전체 대역이 있지만,
      // 오디오 신호는 실수이므로 대칭성을 가집니다.
      // 여기서는 단순화를 위해 Positive Frequency 부분만 마스킹하고,
      // (IFFT 결과에 영향을 주려면 대칭인 Negative Frequency 부분도 처리해야
      // 하지만
      //  GCC PHAT 구현체 특성상 Magnitude 정규화가 되어 있어
      //  특정 대역만 살리는게 조금 까다로울 수 있음.
      //  하지만 편의상 저주파/고주파 bin을 0으로 만드는 것만으로도 상당한
      //  효과가 있음)

      // Positive Freq
      this->fft_out_left_[i * 2] = 0;
      this->fft_out_left_[i * 2 + 1] = 0;

      // Negative Freq (Symmetric)
      int mirror_idx = FFT_N - i;
      if (mirror_idx < FFT_N) {
        this->fft_out_left_[mirror_idx * 2] = 0;
        this->fft_out_left_[mirror_idx * 2 + 1] = 0;
      }
    }

    // Mirror Low Cut (Negative side)
    for (int i = 1; i < 6; i++) {
      int mirror_idx = FFT_N - i;
      this->fft_out_left_[mirror_idx * 2] = 0;
      this->fft_out_left_[mirror_idx * 2 + 1] = 0;
    }

    // IFFT
    dsps_fft2r_fc32(this->fft_out_left_, FFT_N);
    dsps_bit_rev_fc32(this->fft_out_left_, FFT_N);

    // ===== GCC Accumulation =====
    // 프레임 단위의 결과는 노이즈에 취약하므로,
    // 여러 프레임(예: 10프레임 = 약 320ms)의 상관관계를 누적하여 SNR을 높임.

    // 초기화 체크
    if (this->accum_count_ == 0) {
      memset(this->gcc_accum_, 0, sizeof(this->gcc_accum_));
    }

    // 누적 (Magnitude는 어차피 정규화되어 있으므로 단순 합산)
    for (int i = 0; i < FFT_N * 2; i++) {
      this->gcc_accum_[i] += this->fft_out_left_[i];
    }
    this->accum_count_++;

    // 충분히 쌓일 때까지 대기
    static const int ACCUM_FRAMES = 10;
    if (this->accum_count_ < ACCUM_FRAMES) {
      return NAN;
    }

    // ===== 5단계: 피크 찾기 (누적된 데이터 사용) =====
    // 4cm 마이크 간격 @ 16kHz = 최대 ~1.87 샘플 지연
    // 따라서 ±2 샘플만 검색
    float max_val = -1e9f; // -infinite
    int best_lag = 0;

    for (int lag = -2; lag <= 2; lag++) {
      int idx = (lag < 0) ? (FFT_N + lag) : lag;
      // 누적된 실수부 사용 (idx * 2)
      // 평균을 낼 필요는 없음 (최대값의 위치만 중요하므로 합산값 그대로 사용)
      float val = this->gcc_accum_[idx * 2];
      if (val > max_val) {
        max_val = val;
        best_lag = lag;
      }
    }

    // ===== 6단계: 피크 보간 (Quadratic Interpolation) =====
    // 이웃 값 가져오기
    auto get_val = [&](int offset) -> float {
      int idx = (offset < 0) ? (FFT_N + offset) : offset;
      return this->gcc_accum_[idx * 2];
    };

    float val_curr = max_val;
    float val_prev = get_val(best_lag - 1);
    float val_next = get_val(best_lag + 1);

    // Quadratic Interpolation
    float denom = 2.0f * (val_prev - 2.0f * val_curr + val_next);
    float delta = 0.0f;

    if (std::abs(denom) > 1e-6f) {
      delta = (val_prev - val_next) / denom;
    }

    // 보정된 Lag
    float true_lag = (float)best_lag + delta;

    if (true_lag > 2.0f)
      true_lag = 2.0f;
    if (true_lag < -2.0f)
      true_lag = -2.0f;

    // ===== 7단계: 각도 변환 =====
    float lag_dist_m = true_lag * SPEED_OF_SOUND / (float)SAMPLE_RATE;
    float sin_val = lag_dist_m / MIC_DISTANCE;

    if (sin_val > 1.0f)
      sin_val = 1.0f;
    if (sin_val < -1.0f)
      sin_val = -1.0f;

    float angle_rad = asinf(sin_val);
    float angle_deg = angle_rad * 180.0f / (float)M_PI;

    // 방향 보정
    // 하드웨어 배치상 "오른쪽" 소리가 "Positive Lag"를 발생시킴 (Ch1=R, Ch3=L
    // 추정) 따라서 Lag > 0 이면 Positive Angle(Right)이 되어야 함. 기존의
    // -angle_deg 반전을 제거. 또한 4cm의 좁은 간격과 회절 효과로 각도가
    // 과소평가되는 경향이 있어 보정(1.5배)

    // 90도 클리핑
    if (angle_deg > 90.0f)
      angle_deg = 90.0f;
    if (angle_deg < -90.0f)
      angle_deg = -90.0f;

    // ===== 8단계: 스무딩 (Exponential Moving Average) =====
    // 누적을 이미 했으므로 Alpha를 좀 더 민감하게 설정 (0.2 -> 0.6)
    // 반응성을 높임 (이미 300ms 지연되었으므로)
    const float alpha = 0.6f;

    if (std::abs(this->last_output_angle_) < 0.001f &&
        this->last_output_time_ms_ == 0) {
      this->last_output_angle_ = angle_deg;
    } else {
      this->last_output_angle_ =
          (alpha * angle_deg) + ((1.0f - alpha) * this->last_output_angle_);
    }

    this->last_output_time_ms_ = 1;

    // 리셋
    this->accum_count_ = 0;

    // 로그 항상 출력 (업데이트 시점)
    ESP_LOGI(DOA_TAG, "Lag: %.2f (Int:%d) -> Raw: %.1f deg -> Smooth: %.1f deg",
             true_lag, best_lag, angle_deg, this->last_output_angle_);

    return this->last_output_angle_;
  }

  // New Members for Accumulation
  float gcc_accum_[FFT_N * 2];
  int accum_count_{0};

  // Clean up: Reset function not needed for voting anymore
  void reset_votes_() {}
};
