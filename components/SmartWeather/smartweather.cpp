// ESP12F    RFM69CW
// ------------------
// GPIO-5    DIO0
// GPIO-15   NSS
// GPIO-14   SCK
// GPIO-13   MOSI
// GPIO-12   MISO


// FSK weather station receiver
// Frequency 869850 khz
// frequency deviation +/- 60 khz
// data rate 4800
// 
// Package definition:
// Preamble 3 bytes 0xAA, 2 Sync words 0x2D 0xD4, payload 11 bytes
// repeated two times (identical packages) per transmission every 29 seconds


// RFM69 RX-buffer content:
// ************************
// Combined sensor:
// >>>>>>>>>>>>>>>>
// id hh CH ?ttt aa gg sb 4001 crc
// 44 14 0D 4128 30 0D 04 4001 EA
//  0  1  2  3 4  5  6  7  8 9 10
//
// id ..... id of the remote unit. Changes randomly each time the batteries are removed. Is needed to identify "your" unit.
// hh ..... humidity -> uint8 0-99
// CH ..... sensor chanel id = 0x0D
// ttt .... temperature  > temperature is stored as a 16bit integer holding the temperature in degree celcius*10
// aa ..... wind average > A uint16 holding the wind avarage in m/s*5
// gg ..... wind gust  > A uint16 holding the wind gust in m/s*5
// s ...... battery > This uint8 is 00 when battery is ok, 10 when battery is low
// b ...... bearing (cw 0-15) > The wind bearing in 16 clockwise steps (0 = north, 4 = east, 8 = south and C = west)
// crc .... CRC checksum > Poly 0x31, init 0xff, revin&revout, xorout 0x00. Like Maxim 1-wire but with a 0xff init value
//
// Rain sensor:
// >>>>>>>>>>>>>>>>
// id 64 CH ?2BE FF FF sF rrrr crc
// 83 64 10 02BE 30 0D 0F 00AD 16
//  0  1  2  3 4  5  6  7  8 9 10
//
// id ..... id of the remote unit. Changes randomly each time the batteries are removed. Is needed to identify "your" unit.
// CH ..... sensor channel id = 0x10
// s ...... battery > This uint8 is 00 when battery is ok, 10 when battery is low
// rrrr ... rainfall > A uint32 holding accumulated rainfall in 1/4mm
// crc .... CRC checksum > Poly 0x31, init 0xff, revin&revout, xorout 0x00. Like Maxim 1-wire but with a 0xff init value

#include "smartweather.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

// #include <vector>

#define TOINT(type, v, bits) (((struct {type val: bits;}){v}).val)
                    
namespace esphome {
namespace smartweather {

static const char *const TAG = "smartweather";

static const uint8_t CHANNEL_MAIN = 0x0D: //combined outdoor sensor
static const uint8_t CHANNEL_RAIN = 0x10; //rain sensor

static const char *const DIRECTIONS[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                                          "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};

float SmartWeatherComponent::get_setup_priority() const { return setup_priority::DATA; }

void SmartWeatherComponent::setup() {
  ESP_LOGI(TAG, "Setting up SmartWeather");
  rfm.initialize();
}

void SmartWeatherComponent::loop() {
  if (rfm.receiveDone()) {
    ESP_LOGD ("rfm", "PAYLOAD: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X CRC: %02X RSSI: %d dB",
                    rfm.DATA[0],rfm.DATA[1],rfm.DATA[2],rfm.DATA[3],rfm.DATA[4],
                    rfm.DATA[5],rfm.DATA[6],rfm.DATA[7],rfm.DATA[8],rfm.DATA[9], rfm.DATA[10], rfm.rssi);
    if (!rfm.payloadIsValid)
      this->decode_and_publish_();
  }
}

void SmartWeatherComponent::decode_and_publish() {
  uint8_t id = rfm.DATA[0]; //random after battery change
  uint8_t channel = rfm.DATA[2];
  int16_t rssi = rfm.rssi;
  bool battery = rfm.DATA[7] & 0x80; // 0=ok, 1=low
  uint8_t humidity;
  float temperature, wind_speed, wind_gust, wind_direction_degrees, rain_total;
  static float prevrain_total = NAN;

  switch (channel) {
    
    case CHANNEL_MAIN:
      humidity = rfm.DATA[1]; // %rel
      temperature = float(TOINT(int, ((rfm.DATA[3] & 0x0F) << 8) + rfm.DATA[4], 12)) / 10.0;
      wind_speed = float(rfm.DATA[5]) / 5.0;   // m/s;
      wind_gust = float(rfm.DATA[6])  / 5.0;  // m/s;
      wind_direction_degrees = float(rfm.DATA[7] & 0x0F) * 22.5; // 0..360°;

      ESP_LOGD(TAG, "Main ID %d", id);
      if (this->main_rssi_ != nullptr)
        this->main_id_->publish_state(id);
      ESP_LOGD(TAG, "Main RSSI %d", rssi);
      if (this->main_rssi_ != nullptr)
        this->main_rssi_->publish_state(rssi);
      ESP_LOGD(TAG, "Main Battery %d", battery);
      if (this->main_battery_ != nullptr)
        this->main_battery_->publish_state(battery);
      if ((temperature > -25.0) && (temperature < 55.0)) {
        ESP_LOGD(TAG, "Temperature %f", temperature);
        if (this->humidity_ != nullptr)
          this->temperature_->publish_state(temperature);
      }
      if (humidity <= 100) {
        ESP_LOGD(TAG, "Humidity %d", humidity);
        if (this->humidity_ != nullptr)
          this->humidity_->publish_state(humidity);
      }
      if (wind_speed < 20.0) {
        ESP_LOGD(TAG, "Wind direction degrees %f", wind_direction_degrees);
        if (this->wind_direction_ != nullptr)
          this->wind_direction_->publish_state(wind_direction_degrees);
        ESP_LOGD(TAG, "Wind speed %f", wind_speed);
        if (this->wind_speed_ != nullptr)
          this->wind_speed_->publish_state(wind_speed);
        ESP_LOGD(TAG, "Wind gust %f", wind_gust);
        if (this->wind_gust_ != nullptr)
          this->wind_gust_->publish_state(wind_gust);
      }
      break;
    
    case CHANNEL_RAIN:
      rain_total = float((rfm.DATA[8] << 8) + rfm.DATA[9]) * 0.25; 

      ESP_LOGD(TAG, "Rain ID %d", id);
      if (this->rain_rssi_ != nullptr)
        this->rain_id_->publish_state(id);
      ESP_LOGD(TAG, "Rain RSSI %d", rssi);
      if (this->rain_rssi_ != nullptr)
        this->rain_rssi_->publish_state(rssi);
      ESP_LOGD(TAG, "Rain Battery %d", battery);
      if (this->rain_battery_ != nullptr)
        this->rain_battery_->publish_state(battery);
      if (isnan(prevrain_total) || ((rain_total != prevrain_total) && ((rain_total - prevrain_total) < 10.00))) {
        prevrain_total = rain_total;
        if (this->rain_total_ != nullptr)
        this->rain_total_->publish_state(rain_total);
      }
      break;

    default:
      ESP_LOGD (TAG, "Unknown channel %d", channel);
      break;
  } 
}

std::string SmartWeatherComponent::get_wind_cardinal_direction() const { return this->wind_cardinal_direction_; }

}  // namespace smartweather
}  // namespace esphome