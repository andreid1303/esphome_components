#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary/binary.h"
#include "rfm69.h"

namespace esphome {
namespace smartweather {

class SmartWeather : public Component {
  // float get_setup_priority() const override { return esphome::setup_priority::BUS; }

  public:
    RFM69() rfm;
    void setup() override;
    void loop() override;

    std::string get_wind_cardinal_direction() const;
    void set_main_id(sensor::Sensor *main_id) { main_id_ = main_id; }
    void set_main_rssi(sensor::Sensor *main_rssi) { main_rssi_ = main_rssi; }
    void set_main_battery(binary::BinarySensor *main_battery) { main_battery_ = main_battery; }
    void set_temperature(sensor::Sensor *temperature) { temperature_ = temperature; }
    void set_humidity(sensor::Sensor *humidity) { humidity_ = humidity; }
    void set_wind_direction_degrees(sensor::Sensor *wind_direction_degrees) {
      wind_direction_degrees_ = wind_direction_degrees; 
    }   
    void set_wind_speed(sensor::Sensor *wind_speed) { wind_speed_ = wind_speed; }
    void set_wind_gust(sensor::Sensor *wind_gust) { wind_speed_gust_ = wind_gust; }
    
    void set_rain_id(sensor::Sensor *rain_id) { rain_id_ = rain_id; }
    void set_rain_rssi(sensor::Sensor *rain_rssi) { rain_rssi_ = rain_rssi; }
    void set_rain_battery(binary::BinarySensor *rain_battery_binary) { rain_battery_ = rain_battery; }
    void set_rain_total(sensor::Sensor *rain_total) { rain_total_ = rain_total; }

    float get_setup_priority() const override;

  protected:
    void decode_and_publish_();

    std::string wind_cardinal_direction_;
    sensor::Sensor *main_id_{nullptr};
    sensor::Sensor *main_rssi_{nullptr};
    binary::BinarySensor *main_battery_{nullptr};
    sensor::Sensor *temperature_{nullptr};
    sensor::Sensor *humidity_{nullptr};
    sensor::Sensor *wind_direction_degrees_{nullptr};
    sensor::Sensor *wind_speed_{nullptr};
    sensor::Sensor *wind_gust_{nullptr};

    sensor::Sensor *rain_id_{nullptr};
    sensor::Sensor *rain_rssi_{nullptr};
    binary::BinarySensor *rain_battery_{nullptr};
    sensor::Sensor *rain_{nullptr};
};

}  // namespace smartweatherm69
}  // namespace esphome
