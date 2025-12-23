#include "iotawatt.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <cmath>
#include <algorithm>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "soc/spi_struct.h"
#include "hal/gpio_ll.h"

namespace esphome {
namespace iotawatt {

static const char *TAG = "iotawatt";

struct SampleCycleResult {
  bool success{false};
  bool low_quality{false};
  float v_rms{0.0f};
  float i_rms{0.0f};
  float real_power{0.0f};
  float pf{0.0f};
  float frequency{60.0f};
  int samples{0};
  uint16_t vt_offset{2048};
  uint16_t ct_offset{2048};
};

// SPI Configuration
#define SPI_HOST_0 SPI2_HOST // Shared SPI bus for both ADCs
#define SPI_CLOCK_SPEED_HZ 2000000 // 2MHz

void IoTaWattComponent::setup() {
  ESP_LOGI(TAG, "Setting up IoTaWatt (Single SPI bus)...");

  // --- Initialize SPI Bus 0 (ADC0) ---
  spi_bus_config_t buscfg0 = {};
  buscfg0.mosi_io_num = this->mosi_pin_->get_pin();
  buscfg0.miso_io_num = this->miso_pin_->get_pin();
  buscfg0.sclk_io_num = this->clk_pin_->get_pin();
  buscfg0.quadwp_io_num = -1;
  buscfg0.quadhd_io_num = -1;
  buscfg0.max_transfer_sz = 0;

  esp_err_t ret = spi_bus_initialize(SPI_HOST_0, &buscfg0, SPI_DMA_DISABLED);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init SPI0: %d", ret);
    this->mark_failed();
    return;
  }

  // Initialize CS Pins
  this->cs_pin_0_->setup();
  this->cs_pin_1_->setup();
  this->cs_pin_0_->digital_write(true); // Deselect
  this->cs_pin_1_->digital_write(true); // Deselect

  // --- Add Device 0 (ADC0) ---
  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = SPI_CLOCK_SPEED_HZ;
  devcfg.mode = 0;
  devcfg.spics_io_num = -1; // Manual CS
  devcfg.queue_size = 1;
  
  ret = spi_bus_add_device(SPI_HOST_0, &devcfg, &spi_device_0_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add SPI device 0: %d", ret);
    this->mark_failed();
    return;
  }

  // Initialize Channel Data
  channel_data_.resize(15); // Max 15 channels
  for (auto &data : channel_data_) {
    data.v_rms = 0;
    data.i_rms = 0;
    data.active_power = 0;
    data.pf = 0;
    data.frequency = 0;
    data.count = 0;
    data.mux = portMUX_INITIALIZER_UNLOCKED;
  }

  // Start Sampling Task on Core 1 - run highest priority to make sure nothing interrupts sampling
  xTaskCreatePinnedToCore(sampling_task, "iotawatt_task", 8192, this, configMAX_PRIORITIES - 1, &task_handle_, 1);
}

void IoTaWattComponent::add_input_vt(uint8_t channel, std::string name, float cal, float phase, float turns, bool reverse, sensor::Sensor *voltage_sensor, sensor::Sensor *frequency_sensor, const std::vector<float>& p50, const std::vector<float>& p60) {
  InputConfig input;
  input.channel = channel;
  input.name = name;
  input.type = INPUT_TYPE_VT;
  input.cal = cal;
  input.phase = phase;
  input.turns = turns;
  input.reverse = reverse;
  input.vphase = 0.0f;
  input.phase_table.p50 = p50;
  input.phase_table.p60 = p60;
  input.phase_table.base_phase = phase;
  input.voltage_sensor = voltage_sensor;
  input.frequency_sensor = frequency_sensor;
  inputs_.push_back(input);
}

void IoTaWattComponent::add_input_ct(uint8_t channel, std::string name, float cal, float phase, float turns, bool reverse, bool double_input, float vphase, sensor::Sensor *power_sensor, sensor::Sensor *current_sensor, sensor::Sensor *pf_sensor, const std::vector<float>& p50, const std::vector<float>& p60) {
  InputConfig input;
  input.channel = channel;
  input.name = name;
  input.type = INPUT_TYPE_CT;
  input.cal = cal;
  input.phase = phase;
  input.turns = turns;
  input.reverse = reverse;
  input.double_input = double_input;
  input.vphase = vphase;
  input.phase_table.p50 = p50;
  input.phase_table.p60 = p60;
  input.phase_table.base_phase = phase;
  input.power_sensor = power_sensor;
  input.current_sensor = current_sensor;
  input.pf_sensor = pf_sensor;
  inputs_.push_back(input);
}

void IoTaWattComponent::update() {
  for (const auto &input : inputs_) {
    if (input.channel >= channel_data_.size()) continue;

    ChannelData &data = channel_data_[input.channel];
    
    portENTER_CRITICAL(&data.mux);
    float v = data.v_rms;
    float i = data.i_rms;
    float p = data.active_power;
    float pf = data.pf;
    float freq = data.frequency;
    uint32_t c = data.count;
    portEXIT_CRITICAL(&data.mux);

    ESP_LOGD(TAG, "Ch%d count=%u v=%.2f i=%.2f p=%.2f", input.channel, c, v, i, p);

    if (c > 0) {
      if (input.type == INPUT_TYPE_VT) {
        if (input.voltage_sensor) input.voltage_sensor->publish_state(v);
        if (input.frequency_sensor) input.frequency_sensor->publish_state(freq);
      } else if (input.type == INPUT_TYPE_CT) {
        if (input.power_sensor) input.power_sensor->publish_state(p);
        if (input.current_sensor) input.current_sensor->publish_state(i);
        if (input.pf_sensor) input.pf_sensor->publish_state(pf);
      }
    } else {
      ESP_LOGW(TAG, "Ch%d has no data (count=0)", input.channel);
    }
  }
}

void IoTaWattComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "IoTaWatt:");
  LOG_PIN("  CLK Pin: ", this->clk_pin_);
  LOG_PIN("  MOSI Pin: ", this->mosi_pin_);
  LOG_PIN("  MISO Pin: ", this->miso_pin_);
  LOG_PIN("  CS Pin 0: ", this->cs_pin_0_);
  LOG_PIN("  CS Pin 1: ", this->cs_pin_1_);
  for (const auto &input : inputs_) {
    ESP_LOGCONFIG(TAG, "  Input %d: %s (%s)", input.channel, input.name.c_str(), input.type == INPUT_TYPE_VT ? "VT" : "CT");
  }
}

static uint16_t read_adc(spi_device_handle_t spi, int cs_pin, uint8_t channel) {
  static spi_transaction_t t;
  static bool initialized = false;
  
  if (!initialized) {
      t.length = 24; // 3 bytes
      t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
      t.tx_data[2] = 0x00;
      initialized = true;
  }
  
  t.tx_data[0] = 0x06 | ((channel >> 2) & 0x01);
  t.tx_data[1] = (channel & 0x03) << 6;

  GPIO.out_w1tc = (1 << cs_pin);
  
  esp_err_t ret = spi_device_polling_transmit(spi, &t);
  
  GPIO.out_w1ts = (1 << cs_pin);
  
  if (ret != ESP_OK) return 0;

  return ((t.rx_data[1] & 0x0F) << 8) | t.rx_data[2];
}

// Read Aref from ADC1 Ch0
static float read_aref(spi_device_handle_t spi, int cs_pin) {
  uint16_t adc_value = read_adc(spi, cs_pin, 0);
  
  if (adc_value == 4095 || adc_value == 0) {
    return 3.3f;
  }
  
  const float VrefVolts = 2.5f;
  return VrefVolts * 4096.0f / adc_value;
}

// Lookup phase from table based on current
static float lookup_phase(const std::vector<float>& table, float current) {
  if (table.empty()) {
    return 0.0f;
  }
  
  // Table format: [phase1, current1, phase2, current2, ...]
  // Find the bracket where current falls
  for (size_t i = 0; i < table.size() - 2; i += 2) {
    float curr_threshold = table[i + 1];
    if (current <= curr_threshold) {
      return table[i];
    }
  }
  
  // Return last phase if beyond table range
  if (table.size() >= 2) {
    return table[table.size() - 2];
  }
  
  return 0.0f;
}

// Get phase correction based on load and frequency
static float get_phase(const InputConfig& config, float rms_current, float frequency) {
  float phase = config.phase_table.base_phase;
  
  // Only use lookup tables if they have data
  if (frequency < 55.0f && config.phase_table.p50.size() >= 2) {
    phase = lookup_phase(config.phase_table.p50, rms_current);
  } else if (frequency >= 55.0f && config.phase_table.p60.size() >= 2) {
    phase = lookup_phase(config.phase_table.p60, rms_current);
  }
  
  // Safety check for invalid values
  if (!std::isfinite(phase)) {
    phase = config.phase_table.base_phase;
  }
  
  return phase;
}

static IRAM_ATTR __attribute__((noinline)) uint16_t read_adc_fast(int cs_pin, uint8_t channel) {
    GPIO.out_w1tc = (1 << cs_pin);

    uint32_t data_word = (0x06 | ((channel >> 2) & 0x01)) | 
                         (((channel & 0x03) << 6) << 8); 

    GPSPI2.ms_dlen.ms_data_bitlen = 23; // 24 bits
    GPSPI2.data_buf[0] = data_word;

    // DELAY (Required for MCP3208 tCSS > 100ns)
    // ESP32-S3 @ 240MHz is ~4ns per instruction. 20 nops + 2 instructions = ~100ns
    asm volatile("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");
    asm volatile("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");

    GPSPI2.cmd.usr = 1;

    while (GPSPI2.cmd.usr);
    GPIO.out_w1ts = (1 << cs_pin);
    uint32_t val = GPSPI2.data_buf[0];
    return (((val >> 8) & 0x0F) << 8) | ((val >> 16) & 0xFF);
}

static SampleCycleResult sample_cycle(const InputConfig &vt_config,
                                      const InputConfig &ct_config, 
                                      int vt_cs_pin, uint8_t vt_adc_chan,
                                      int ct_cs_pin, uint8_t ct_adc_chan,
                                      uint16_t &vt_offset, uint16_t &ct_offset,
                                      float &frequency, float aref) {
  SampleCycleResult result;
  result.vt_offset = vt_offset;
  result.ct_offset = ct_offset;

  constexpr int MAX_SAMPLES = 10000;
  std::vector<int16_t> v_samples(MAX_SAMPLES + 1);
  std::vector<int16_t> i_samples(MAX_SAMPLES + 1);

  // Scan VT for 20ms to find midpoint and confirm signal
  uint16_t min_val = 4096;
  uint16_t max_val = 0;
  int64_t scan_end = esp_timer_get_time() + 20000; // 20ms
  while (esp_timer_get_time() < scan_end) {
    uint16_t r = read_adc_fast(vt_cs_pin, vt_adc_chan);
    if (r < min_val) min_val = r;
    if (r > max_val) max_val = r;
  }
  bool has_signal = (max_val - min_val) > 100; // ~80mV threshold
  vt_offset = (min_val + max_val) / 2;
  result.vt_offset = vt_offset;
  if (!has_signal) {
    return result;
  }

  // Prime voltage
  int16_t rawV = static_cast<int16_t>(read_adc_fast(vt_cs_pin, vt_adc_chan)) - static_cast<int16_t>(vt_offset);
  int16_t lastV = rawV;
  int16_t rawI = 0;

  int16_t crossLimit = 3; // one full cycle => 3 crossings
  int16_t crossCount = 0;
  int16_t crossGuard = 4;
  int16_t midCrossSamples = 0;
  bool Vsensed = false;

  int samples = 0;
  int64_t start_us = esp_timer_get_time();
  const int64_t timeout_us = 12000; // slightly more than half-cycle @50Hz
  int64_t firstCrossUs = 0;
  int64_t lastCrossUs = 0;

  do {
    // Sample current (I)
    rawI = static_cast<int16_t>(read_adc_fast(ct_cs_pin, ct_adc_chan)) - static_cast<int16_t>(ct_offset);

    // Average voltage for alignment with last voltage sample
    int16_t avgV = (rawV + lastV) >> 1;
    int16_t storeI = rawI;
    int16_t storeV = avgV;
    lastV = rawV;

    if (crossCount) {
      if (samples >= MAX_SAMPLES) {
        return result; // failure: overrun
      }
      v_samples[samples] = storeV;
      i_samples[samples] = storeI;
      samples++;
    } else {
      if (rawV < -10 || rawV > 10) {
        Vsensed = true;
      }
    }

    crossGuard--;

    // Sample voltage (V)
    rawV = static_cast<int16_t>(read_adc_fast(vt_cs_pin, vt_adc_chan)) - static_cast<int16_t>(vt_offset);

    // Timeout safeguards
    if ((esp_timer_get_time() - start_us) > timeout_us) {
      return result;
    } else if (!crossGuard && !Vsensed) {
      return result;
    }

    // Zero small noise on I
    if (rawI >= -1 && rawI <= 1) rawI = 0;

    // Zero-cross detection with guard
    if (((rawV ^ lastV) & crossGuard) < 0) {
      start_us = esp_timer_get_time();
      crossCount++;
      if (crossCount == 1) {
        firstCrossUs = esp_timer_get_time();
        crossGuard = 10;
      } else if (crossCount == crossLimit) {
        lastCrossUs = esp_timer_get_time();
        v_samples[samples] = (lastV + rawV) >> 1;
        i_samples[samples] = storeI;
        crossGuard = 0;
      } else if (crossCount == ((crossLimit + 1) / 2)) {
        midCrossSamples = samples;
        crossGuard = 10;
      }
    }
  } while (crossCount < crossLimit || crossGuard > 0);

  // Process samples
  int32_t sumV = 0;
  int32_t sumI = 0;
  int64_t sumVsq = 0;
  int64_t sumIsq = 0;
  int64_t sumVI = 0;

  for (int idx = 0; idx < samples; idx++) {
    if (i_samples[idx] == -1 || i_samples[idx] == 1) {
      i_samples[idx] = 0;
    }
    sumV += v_samples[idx];
    sumI += i_samples[idx];

    int16_t v_val = vt_config.reverse ? -v_samples[idx] : v_samples[idx];
    int16_t i_val = ct_config.reverse ? -i_samples[idx] : i_samples[idx];

    sumVsq += static_cast<int64_t>(v_val) * v_val;
    sumIsq += static_cast<int64_t>(i_val) * i_val;
    sumVI += static_cast<int64_t>(i_val) * v_val;
  }

  if (samples == 0 || firstCrossUs == 0 || lastCrossUs <= firstCrossUs) {
    return result;
  }

  // Reject very low sample count
  if (samples < 320) {
    result.low_quality = true;
    return result;
  }

  // Reject unbalanced half-cycles
  if (std::abs(samples - (midCrossSamples * 2)) > 8) {
    result.low_quality = true;
    return result;
  }

  // Update offsets slowly
  int32_t v_drift = sumV / samples;
  int32_t i_drift = sumI / samples;
  int new_vt_offset = static_cast<int>(vt_offset) + v_drift / 10;
  int new_ct_offset = static_cast<int>(ct_offset) + i_drift / 10;
  vt_offset = static_cast<uint16_t>(std::min(4095, std::max(0, new_vt_offset)));
  ct_offset = static_cast<uint16_t>(std::min(4095, std::max(0, new_ct_offset)));
  result.vt_offset = vt_offset;
  result.ct_offset = ct_offset;

  // Frequency estimate
  float Hz = 1000000.0f / static_cast<float>(lastCrossUs - firstCrossUs);
  frequency = (0.9f * frequency) + (0.1f * Hz);
  result.frequency = frequency;

  // Ratios
  float v_ratio = vt_config.cal * 13.0f * aref / 4096.0f;
  float i_ratio = ct_config.cal * aref / 4096.0f;

  float v_rms_initial = v_ratio * sqrt(static_cast<double>(sumVsq) / samples);
  float i_rms_initial = i_ratio * sqrt(static_cast<double>(sumIsq) / samples);

  float vt_phase = get_phase(vt_config, v_rms_initial, frequency);
  float ct_phase = get_phase(ct_config, i_rms_initial, frequency);
  if (!std::isfinite(vt_phase)) vt_phase = 0.0f;
  if (!std::isfinite(ct_phase)) ct_phase = 0.0f;

  float phase_correction = (ct_phase - vt_phase - ct_config.vphase) * samples / 360.0f;
  if (!std::isfinite(phase_correction) || std::abs(phase_correction) > samples) {
    phase_correction = 0.0f;
  }
  int step_correction = static_cast<int>(phase_correction);
  float step_fraction = phase_correction - step_correction;
  if (step_fraction < 0) {
    step_correction--;
    step_fraction += 1.0f;
  }

  // Phase-corrected sums
  v_samples[samples] = v_samples[0]; // wrap
  int64_t sum_vi_corrected = 0;
  int64_t sum_v_sq_corrected = 0;
  int64_t sum_i_sq_corrected = 0;

  int v_index = (samples + step_correction) % samples;
  for (int idx = 0; idx < samples; idx++) {
    int16_t i_val = ct_config.reverse ? -i_samples[idx] : i_samples[idx];
    int16_t v_base = vt_config.reverse ? -v_samples[v_index] : v_samples[v_index];
    int16_t v_next = vt_config.reverse ? -v_samples[(v_index + 1) % (samples + 1)] : v_samples[(v_index + 1) % (samples + 1)];

    int16_t v_val = v_base + static_cast<int16_t>(step_fraction * (v_next - v_base));

    sum_v_sq_corrected += static_cast<int64_t>(v_val) * v_val;
    sum_i_sq_corrected += static_cast<int64_t>(i_val) * i_val;
    sum_vi_corrected += static_cast<int64_t>(v_val) * i_val;

    v_index = (v_index + 1) % samples;
  }

  float v_rms = v_ratio * sqrt(static_cast<double>(sum_v_sq_corrected) / samples);
  float i_rms = i_ratio * sqrt(static_cast<double>(sum_i_sq_corrected) / samples);
  float real_power = v_ratio * i_ratio * (static_cast<double>(sum_vi_corrected) / samples);

  if (v_rms < 1.0f) v_rms = 0.0f;
  if (i_rms < 0.1f) i_rms = 0.0f;
  if (std::abs(real_power) < 1.0f) real_power = 0.0f;

  float apparent_power = v_rms * i_rms;
  float pf = 0.0f;
  if (apparent_power > 0.1f) {
    pf = real_power / apparent_power;
    if (pf > 1.0f) pf = 1.0f;
    if (pf < -1.0f) pf = -1.0f;
  }

  result.v_rms = v_rms;
  result.i_rms = i_rms;
  result.real_power = real_power;
  result.pf = pf;
  result.samples = samples;
  result.success = true;
  return result;
}

void IoTaWattComponent::sampling_task(void *arg) {
  IoTaWattComponent *self = (IoTaWattComponent *)arg;
  
  // Identify VT channel
  int vt_channel_idx = -1;
  for (int i = 0; i < self->inputs_.size(); i++) {
    if (self->inputs_[i].type == INPUT_TYPE_VT) {
      vt_channel_idx = i;
      break;
    }
  }

  if (vt_channel_idx == -1) {
    ESP_LOGE(TAG, "No VT channel found!");
    vTaskDelete(NULL);
    return;
  }

  InputConfig &vt_config = self->inputs_[vt_channel_idx];
  uint8_t vt_adc_chan = vt_config.channel % 8;
  int vt_cs_pin = self->cs_pin_0_->get_pin(); // VT is always on ADC0

  static uint16_t vt_offset = 2048;
  static uint16_t ct_offsets[15] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048,
                                     2048, 2048, 2048, 2048, 2048, 2048, 2048};

  static uint32_t last_log_ms = 0;

  while (true) {
    spi_device_acquire_bus(self->spi_device_0_, portMAX_DELAY);
    // Ensure SPI hardware registers are primed for VT before fast path
    read_adc(self->spi_device_0_, vt_cs_pin, vt_adc_chan);

    float vt_rms_for_sweep = 0.0f;
    bool vt_rms_valid = false;
    float frequency_for_vt = 0.0f;
    bool frequency_valid = false;

    // Loop through all CT inputs
    for (auto &input : self->inputs_) {
      if (input.type == INPUT_TYPE_VT) continue; // Skip VT

      uint8_t ct_adc_chan = input.channel >= 8 ? input.channel - 7 : input.channel;
      int ct_cs_pin = (input.channel < 8) ? self->cs_pin_0_->get_pin() : self->cs_pin_1_->get_pin();

      float aref = read_aref(self->spi_device_0_, self->cs_pin_1_->get_pin());

      SampleCycleResult res = sample_cycle(vt_config, input, 
                                           vt_cs_pin, vt_adc_chan,
                                           ct_cs_pin, ct_adc_chan,
                                           vt_offset, ct_offsets[input.channel],
                                           self->frequency_, aref);

      if (!res.success || res.low_quality) {
        continue;
      }

      float scale = input.double_input ? 2.0f : 1.0f;

      ChannelData &data = self->channel_data_[input.channel];
      portENTER_CRITICAL(&data.mux);
      data.v_rms = res.v_rms;
      data.i_rms = res.i_rms * scale;
      data.active_power = res.real_power * scale;
      data.pf = res.pf;
      data.count++;
      portEXIT_CRITICAL(&data.mux);

      if (!vt_rms_valid) {
        vt_rms_for_sweep = res.v_rms;
        vt_rms_valid = true;
      }
      frequency_for_vt = res.frequency;
      frequency_valid = true;

      // Occasional instrumentation
      uint32_t now = millis();
      if (now - last_log_ms > 5000) {
        last_log_ms = now;
        ESP_LOGD(TAG, "Ch%d samples=%d freq=%.2fHz vt_off=%u ct_off=%u Vrms=%.2f Irms=%.2f Pf=%.2f",
                 input.channel, res.samples, res.frequency, res.vt_offset, res.ct_offset,
                 res.v_rms, res.i_rms, res.pf);
      }
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    if (vt_rms_valid || frequency_valid) {
      ChannelData &vt_data = self->channel_data_[vt_config.channel];
      portENTER_CRITICAL(&vt_data.mux);
      if (vt_rms_valid) {
        vt_data.v_rms = vt_rms_for_sweep;
        vt_data.count++;
      }
      if (frequency_valid) {
        vt_data.frequency = frequency_for_vt;
      }
      portEXIT_CRITICAL(&vt_data.mux);
    }

    spi_device_release_bus(self->spi_device_0_);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

}  // namespace iotawatt
}  // namespace esphome
