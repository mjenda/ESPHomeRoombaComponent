#pragma once

#include "esphome.h"

#include "Roomba.h"
#include "Status.h"

// Sensor decorator, which publishes state if it has changed or has not been
// initialized yet
template <typename Base>
class RoombaSensor : public Base {
 public:
  template <typename T>
  void publishState(T stateToPublish) {
    if (Base::state != stateToPublish || !Base::has_state()) {
      Base::publish_state(stateToPublish);
    }
  }
};

class RoombaComponent : public esphome::PollingComponent,
                        public esphome::mqtt::CustomMQTTDevice,
                        public esphome::api::CustomAPIDevice {
 private:
  uint8_t brcPin;
  uint32_t updateInterval;
  std::string stateTopic;
  std::string commandTopic;
  Roomba roomba;
  Status status_;

 public:
  RoombaSensor<esphome::sensor::Sensor>* distanceSensor;
  RoombaSensor<esphome::sensor::Sensor>* voltageSensor;
  RoombaSensor<esphome::sensor::Sensor>* currentSensor;
  RoombaSensor<esphome::sensor::Sensor>* chargeSensor;
  RoombaSensor<esphome::sensor::Sensor>* capacitySensor;
  RoombaSensor<esphome::binary_sensor::BinarySensor>* chargingBinarySensor;
  RoombaSensor<esphome::binary_sensor::BinarySensor>* dockedBinarySensor;
  RoombaSensor<esphome::binary_sensor::BinarySensor>* cleaningBinarySensor;
  RoombaSensor<esphome::binary_sensor::BinarySensor>* sleepingBinarySensor;

  static RoombaComponent* instance(const std::string& stateTopic,
                                   const std::string& commandTopic,
                                   uint8_t brcPin,
                                   uint32_t updateInterval);

  void setup() override;
  void update() override;
  void wakeUp();
  void on_message(const std::string& payload);
  void onCustomCommand(std::string str);

 private:
  RoombaComponent(const std::string& stateTopic,
                  const std::string& commandTopic,
                  uint8_t brcPin,
                  uint32_t updateInterval);
};
