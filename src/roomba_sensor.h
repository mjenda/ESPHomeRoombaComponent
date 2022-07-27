#pragma once

#include "esphome.h"



// Sensor decorator, which publishes state if it has changed or has not been
// initialized yet
template <typename Base>
class RoombaSensorStatePublisher : public Base {
 public:
  template <typename T>
  void publishState(T state_to_publish) {
    if (Base::state != state_to_publish || !Base::has_state()) {
      Base::publish_state(state_to_publish);
    }
  }
};

using RoombaSensor = RoombaSensorStatePublisher<esphome::sensor::Sensor>;
using RoombaBinarySensor = RoombaSensorStatePublisher<esphome::binary_sensor::BinarySensor>;
