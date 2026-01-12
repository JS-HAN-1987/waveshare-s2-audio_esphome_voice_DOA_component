# ESP-SR DOA Component for ESPHome

This component integrates the Espressif ESP-SR Audio Front-End (AFE) with Direction of Arrival (DOA) into ESPHome.

## Features
- **Wake Word Detection**: Uses ESP-SR WakeNet (e.g., `wakenet_model`).
- **Direction of Arrival (DOA)**: Reports the angle (0-360°) of the sound source when the wake word is detected.
- **Voice Command (Optional)**: Integration with MultiNet (placeholder).

## Installation

Add this repository as an external component in your ESPHome YAML configuration:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/JS-HAN-1987/waveshare-s2-audio_esphome_voice_DOA_component
      ref: main
    components: [ esp_sr_doa ]
```

## Configuration

```yaml
esp_sr_doa:
  id: my_esp_sr
  mic_i2s_id: i2s_input # ID of your i2s_audio input component
  
  wake_word_sensor:
    name: "ESP-SR Wake Word"
    
  doa_sensor:
    name: "DOA Angle"
    filters:
      - sliding_window_moving_average:
          window_size: 3
          send_every: 1
          
  # Optional: WakeNet model config (not yet fully dynamic in this version, uses partition 'model')
```

## Dependencies
- Requires `espressif/esp-sr` component. This is usually handled by `idf_component.yml` if building with ESP-IDF.
- This component assumes `i2s_audio` is configured for the microphone source.

## Hardware Support
- Tested with ESP32-S3.
- Default configuration assumes a 2-microphone array (e.g., ESP32-S3-BOX, Waveshare Audio S3).

## License
MIT (or your preferred license)
