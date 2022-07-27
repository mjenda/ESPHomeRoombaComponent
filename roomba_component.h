#pragma once

#include "esphome.h"

#include "Roomba.h"
#include "roomba_sensor.h"
#include "status.h"

class RoombaComponent : public esphome::PollingComponent,
                        public esphome::mqtt::CustomMQTTDevice,
                        public esphome::api::CustomAPIDevice {
 public:
  static RoombaComponent* instance(const std::string& state_topic,
                                   const std::string& command_topic,
                                   uint8_t brc_bin,
                                   uint32_t update_interval);

  void setup() override;
  void update() override;
  void wakeUp(bool initial_wake = false);
  void onCommand(const std::string& payload);
  void onCustomCommand(std::string str);

  RoombaSensor distance_sensor_;
  RoombaSensor voltage_sensor_;
  RoombaSensor current_sensor_;
  RoombaSensor charge_sensor_;
  RoombaSensor capacity_sensor_;
  RoombaSensor distance_since_start_sensor_;
  RoombaBinarySensor charging_binary_sensor_;
  RoombaBinarySensor docked_binary_sensor_;
  RoombaBinarySensor cleaning_binary_sensor_;
  RoombaBinarySensor sleeping_binary_sensor_;

 private:
  RoombaComponent(const std::string& state_topic,
                  const std::string& command_topic,
                  uint8_t brc_pin,
                  uint32_t update_interval);

  uint8_t brc_pin_;
  uint32_t update_interval_;
  std::string state_topic_;
  std::string command_topic_;
  Roomba roomba_;
  Status status_;
};
