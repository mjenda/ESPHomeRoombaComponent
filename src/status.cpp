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

  static auto request = GetStatusRequest();
  static auto status = GetStatusMessage();

  const bool result = roomba_.getSensorsList(request.data(), request.size(),
                                             status.data(), status.size());

  if (!ValidateNewStatus(result, status)) {
    return false;
  }

  status_ = status;

  UpdateDistance();

  return true;
}

int16_t Status::GetDistance() const {
  return status_[0] * 256 + status_[1];
}

uint32_t Status::GetDistanceSinceStart() const {
  return distance_since_start_ / 10;
}

uint16_t Status::GetVoltage() const {
  return status_[3] * 256 + status_[4];
}

int16_t Status::GetCurrent() const {
  return status_[5] * 256 + status_[6];
}

uint16_t Status::GetCharge() const {
  return status_[7] * 256 + status_[8];
}

uint16_t Status::GetCapacity() const {
  return status_[9] * 256 + status_[10];
}

uint8_t Status::GetCharging() const {
  return status_[2];
}

bool Status::GetCleaningState() const {
  return GetCurrent() < -300;
}

bool Status::GetDockedState() const {
  return GetCurrent() > -50;
}

bool Status::GetChargingState() const {
  const uint8_t charging = GetCharging();
  return charging == Roomba::ChargeStateReconditioningCharging ||
         charging == Roomba::ChargeStateFullChanrging ||
         charging == Roomba::ChargeStateTrickleCharging;
}

bool Status::GetSleepState() const {
  return sleep_mode_cunter_ > 2;
}

float Status::GetBatteryLevel() const {
  const float value = 100.0 * ((1.0 * GetCharge()) / (1.0 * GetCapacity()));
  return round(value * 100) / 100;
}

std::string Status::GetState() const {
  return GetCleaningState()   ? "cleaning"
         : GetDockedState()   ? "docked"
         : GetChargingState() ? "idle"
                              : "idle";
}

void Status::UpdateDistance() {
  if (GetCleaningState()) {
    distance_since_start_ += std::abs(GetDistance());
  } else {
    distance_since_start_ = 0;
  }
}

bool Status::IsStatusValid(const Status::StatusMessageType& status) const {
  // Unfortunately there is no checksum, so just simple chcek if voltage or
  // capacity is not zero.
  return !(status[3] * 256 + status[4] == 0 ||
           status[9] * 256 + status[10] == 0);
}

bool Status::ValidateNewStatus(bool result,
                             const Status::StatusMessageType& status) {
  if (!result) {
    ESP_LOGD(TAG, "Unable to read sensors from the roomba");
    sleep_mode_cunter_++;
    return false;
  }

  if (!IsStatusValid(status)) {
    ESP_LOGD(TAG, "Malformed data. Skipping");
    return false;
  }

  sleep_mode_cunter_ = 0;
  return true;
}