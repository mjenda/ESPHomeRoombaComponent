#pragma once

#include "Roomba.h"
#include "constants.h"
#include "esphome.h"

namespace {

constexpr auto GetStatusMessage = []() { return std::array<uint8_t, 11>(); };

constexpr auto GetStatusRequest = []() {
  return std::array<uint8_t, 6>({
      Roomba::SensorDistance,        // 2 bytes, mm, signed
      Roomba::SensorChargingState,   // 1 byte
      Roomba::SensorVoltage,         // 2 bytes, mV, unsigned
      Roomba::SensorCurrent,         // 2 bytes, mA, signed
      Roomba::SensorBatteryCharge,   // 2 bytes, mAh, unsigned
      Roomba::SensorBatteryCapacity  // 2 bytes, mAh, unsigned
  });
};

}  // namespace

class Status {
 public:
  using StatusMessageType = decltype(GetStatusMessage());

  Status(Roomba& roomba);

  float Get(Sensors sensor) const;
  bool Get(BinarySensors sensor) const;
  std::string GetState() const;

  bool OnPendingData();

 private:
  StatusMessageType status_ = GetStatusMessage();
  Roomba& roomba_;
  bool isInSleepMode_ = true;
};
