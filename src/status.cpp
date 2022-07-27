#include "Status.h"

using esphome::esp_log_printf_;

static const char* TAG = "component.Roomba.Status";

Status::Status(Roomba& roomba) : roomba_(roomba) {}

bool Status::OnPendingData() {
  ESP_LOGD(TAG, "Attempting to update sensor status_.");

  // Flush serial buffers
  while (Serial.available()) {
    Serial.read();
  }

  auto request = GetStatusRequest();
  auto status = GetStatusMessage();

  isInSleepMode_ = !roomba_.getSensorsList(request.data(), request.size(),
                                           status.data(), status.size());

  if (isInSleepMode_) {
    ESP_LOGD(TAG,
             "Unable to read sensors from the roomba. Roomba is probably in "
             "sleeping mode");
    return false;
  }

  status_ = status;

  return true;
}

float Status::Get(Sensors sensor) const {
  switch (sensor) {
    case Sensors::Distance:
      return status_[0] * 256 + status_[1];
    case Sensors::Voltage:
      return status_[3] * 256 + status_[4];
    case Sensors::Current:
      return status_[5] * 256 + status_[6];
    case Sensors::Charge:
      return status_[7] * 256 + status_[8];
    case Sensors::Capacity:
      return status_[9] * 256 + status_[10];
    case Sensors::Charging:
      return status_[2];
    case Sensors::BatteryLevel: {
      const float value =
          100.0 * ((1.0 * Get(Sensors::Charge)) / (1.0 * Get(Sensors::Capacity)));
      return round(value * 100) / 100;
    }
    default:
      return 0.0;
  }
}

bool Status::Get(BinarySensors sensor) const {
  switch (sensor) {
    case BinarySensors::CleaningState:
      return Get(Sensors::Current) < -300;
    case BinarySensors::ChargingState: {
      const uint8_t charging = Get(Sensors::Charging);
      return charging == Roomba::ChargeStateReconditioningCharging ||
             charging == Roomba::ChargeStateFullChanrging ||
             charging == Roomba::ChargeStateTrickleCharging;
    }
    case BinarySensors::DockedState:
      return Get(Sensors::Current) > -50;
    case BinarySensors::SleepState:
      return isInSleepMode_;
    default:
      return false;
  }
}

std::string Status::GetState() const {
  return Get(BinarySensors::CleaningState)   ? "cleaning"
         : Get(BinarySensors::DockedState)   ? "docked"
         : Get(BinarySensors::ChargingState) ? "idle"
                                             : "idle";
}