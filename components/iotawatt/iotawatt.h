#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>
#include <string>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace esphome {
namespace iotawatt {

enum InputType {
  INPUT_TYPE_VT,
  INPUT_TYPE_CT
};

struct PhaseTable {
  std::vector<float> p50;  // 50Hz: [phase1, current1, phase2, current2, ...]
  std::vector<float> p60;  // 60Hz: [phase1, current1, phase2, current2, ...]
  float base_phase;        // Fallback if no table
};

struct InputConfig {
  uint8_t channel;
  std::string name;
  InputType type;
  float cal;
  float phase;
  float turns;
  bool reverse;
  bool double_input;
  float vphase;  // Gross phase correction for 3-phase
  PhaseTable phase_table;
  
  sensor::Sensor *voltage_sensor{nullptr};
  sensor::Sensor *frequency_sensor{nullptr};
  sensor::Sensor *current_sensor{nullptr};
  sensor::Sensor *power_sensor{nullptr};
  sensor::Sensor *pf_sensor{nullptr};
};

class IoTaWattComponent : public PollingComponent {
 public:
  IoTaWattComponent() : PollingComponent(5000) {}
  void setup() override;
  void update() override;
  void dump_config() override;

  void set_clk_pin(InternalGPIOPin *pin) { clk_pin_ = pin; }
  void set_mosi_pin(InternalGPIOPin *pin) { mosi_pin_ = pin; }
  void set_miso_pin(InternalGPIOPin *pin) { miso_pin_ = pin; }
  void set_cs_pin_0(InternalGPIOPin *pin) { cs_pin_0_ = pin; }
  void set_cs_pin_1(InternalGPIOPin *pin) { cs_pin_1_ = pin; }

  void add_input_vt(uint8_t channel, std::string name, float cal, float phase, float turns, bool reverse, sensor::Sensor *voltage_sensor, sensor::Sensor *frequency_sensor, const std::vector<float>& p50, const std::vector<float>& p60);

  void add_input_ct(uint8_t channel, std::string name, float cal, float phase, float turns, bool reverse, bool double_input, float vphase, sensor::Sensor *power_sensor, sensor::Sensor *current_sensor, sensor::Sensor *pf_sensor, const std::vector<float>& p50, const std::vector<float>& p60);

 protected: 
  static void sampling_task(void *arg);
  
  spi_device_handle_t spi_device_0_{nullptr};
  
  TaskHandle_t task_handle_{nullptr};
  
  InternalGPIOPin *clk_pin_;
  InternalGPIOPin *mosi_pin_;
  InternalGPIOPin *miso_pin_;
  InternalGPIOPin *cs_pin_0_;
  InternalGPIOPin *cs_pin_1_;

  std::vector<InputConfig> inputs_;
  // We should accumulate in the task and publish in `loop()` or `update()`.
  
  // Accumulators for passing data from task to loop
  struct ChannelData {
    float v_rms;
    float i_rms;
    float active_power;
    float pf;
    float frequency;
    uint32_t count;
    portMUX_TYPE mux;
  };
  
  std::vector<ChannelData> channel_data_;
  
  uint32_t last_update_{0};
  float frequency_{60.0f};
};

}  // namespace iotawatt
}  // namespace esphome
