#pragma once

#include <Roomba.h>
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

  int16_t GetDistance() const;
  uint32_t GetDistanceSinceStart() const;
  uint16_t GetVoltage() const;
  int16_t GetCurrent() const;
  uint16_t GetCharge() const;
  uint16_t GetCapacity() const;
  uint8_t GetCharging() const;
  bool GetCleaningState() const;
  bool GetChargingState() const;
  bool GetDockedState() const;
  bool GetSleepState() const;
  float GetBatteryLevel() const;
  std::string GetState() const;

  bool OnPendingData();

 private:
  void UpdateDistance();

  StatusMessageType status_ = GetStatusMessage();
  Roomba& roomba_;
  uint32_t distance_since_start_ = 0;
  bool isInSleepMode_ = true;
};
