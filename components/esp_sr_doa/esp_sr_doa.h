#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "gcc_phat_doa.h"


namespace esphome {
namespace esp_sr_doa {

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

  // Shared Logic Engine
  GccPhatDoa doa_engine_;

  float current_doa_{0.0f};
  uint32_t last_process_time_{0};
};

} // namespace esp_sr_doa
} // namespace esphome
