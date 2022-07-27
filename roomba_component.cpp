#include "roomba_component.h"

using esphome::esp_log_printf_;

static const char* TAG = "component.Roomba";

RoombaComponent* RoombaComponent::instance(const std::string& state_topic,
                                           const std::string& command_topic,
                                           uint8_t brc_pin,
                                           uint32_t update_interval) {
  static RoombaComponent* g_instance =
      new RoombaComponent(state_topic, command_topic, brc_pin, update_interval);
  return g_instance;
}

void RoombaComponent::setup() {
  pinMode(brc_pin_, OUTPUT);
  digitalWrite(brc_pin_, HIGH);

  roomba_.start();

  wakeUp();

  subscribe(command_topic_, &RoombaComponent::onCommand);
}

void RoombaComponent::update() {
  if (!status_.OnPendingData()) {
    sleeping_binary_sensor_.publishState(status_.GetSleepState());
    return;
  }

  distance_sensor_.publishState(status_.GetDistance());
  voltage_sensor_.publishState(status_.GetVoltage());
  current_sensor_.publishState(status_.GetCurrent());
  charge_sensor_.publishState(status_.GetCharge());
  capacity_sensor_.publishState(status_.GetCapacity());
  distance_since_start_sensor_.publishState(status_.GetDistanceSinceStart());
  charging_binary_sensor_.publishState(status_.GetCharging());
  docked_binary_sensor_.publishState(status_.GetDockedState());
  cleaning_binary_sensor_.publishState(status_.GetCleaningState());
  sleeping_binary_sensor_.publishState(status_.GetSleepState());

  publish_json(state_topic_, [=](JsonObject root) {
    root["battery_level"] = status_.GetBatteryLevel();
    root["state"] = status_.GetState();
    root["fan_speed"] = "off";
  });
}

void RoombaComponent::wakeUp() {
  if (status_.GetSleepState()) {
    digitalWrite(brc_pin_, HIGH);
    delay(100);
    digitalWrite(brc_pin_, LOW);
    delay(500);
    digitalWrite(brc_pin_, HIGH);
    delay(100);
  }
}

void RoombaComponent::onCommand(const std::string& payload) {
  ESP_LOGD(TAG, "Got values %s", payload.c_str());

  wakeUp();

  if (status_.GetCleaningState()) {
    // Stop previous command if in progress
    roomba_.cover();
    delay(500);
  } else if (status_.GetDockedState()) {
    // Activate green button if docked
    roomba_.dock();
    delay(1000);
  }

  if (payload == "turn_on" || payload == "start") {
    roomba_.cover();
  } else if ((payload == "turn_off" || payload == "stop")) {
    // Already handled when stopping previous command
  } else if (payload == "dock" || payload == "return_to_base") {
    roomba_.dock();
  } else if (payload == "locate") {
    roomba_.playSong(1);
  } else if (payload == "spot" || payload == "clean_spot") {
    roomba_.spot();
  } else {
    ESP_LOGW(TAG, "Received unknown status payload: %s", payload.c_str());
    status_momentary_warning("state", 5000);
  }

  delay(500);
  update();
}
RoombaComponent::RoombaComponent(const std::string& state_topic,
                                 const std::string& command_topic,
                                 uint8_t brc_pin,
                                 uint32_t update_interval)
    : PollingComponent(update_interval),
      brc_pin_(brc_pin),
      state_topic_(state_topic),
      command_topic_(command_topic),
      roomba_(&Serial, Roomba::Baud115200),
      status_(roomba_) {}
