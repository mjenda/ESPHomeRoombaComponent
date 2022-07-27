#include "roomba_component.h"

#include <sstream>

using esphome::esp_log_printf_;

namespace {
std::vector<uint8_t> getCustomCommands(const std::string& input) {
  std::vector<uint8_t> vect;

  std::istringstream ss(input);
  std::string token;

  while (std::getline(ss, token, ',')) {
    int number = atoi(token.c_str());
    if (number != 0 || token == "0")
      vect.push_back(number);
  }

  return vect;
}

}  // namespace

static const char* TAG = "component.Roomba";

RoombaComponent* RoombaComponent::instance(const std::string& state_topic,
                                           const std::string& command_topic,
                                           uint8_t brc_pin,
                                           uint32_t update_interval) {
  static RoombaComponent* INSTANCE =
      new RoombaComponent(state_topic, command_topic, brc_pin, update_interval);
  return INSTANCE;
}

void RoombaComponent::setup() {
  ESP_LOGD(TAG, "Setting up roomba.");
  pinMode(brc_pin_, OUTPUT);
  digitalWrite(brc_pin_, HIGH);

  roomba_.start();

  wakeUp(/*initial_wake=*/true);

  ESP_LOGD(TAG, "Attempting to subscribe to MQTT.");

  subscribe(command_topic_, &RoombaComponent::on_message);
  register_service(&RoombaComponent::onCustomCommand, "command", {"command"});
}

void RoombaComponent::update() {
  status_.OnPendingData();

  distance_sensor_.publishState(status_.GetDistance());
  voltage_sensor_.publishState(status_.GetVoltage());
  current_sensor_.publishState(status_.GetCurrent());
  charge_sensor_.publishState(status_.GetCharge());
  capacity_sensor_.publishState(status_.GetCapacity());
  charging_binary_sensor_.publishState(status_.GetCharging());
  docked_binary_sensor_.publishState(status_.GetChargingState());
  cleaning_binary_sensor_.publishState(status_.GetCleaningState());
  sleeping_binary_sensor_.publishState(status_.GetSleepState());

  // Publish to the state topic a json document; necessary for the 'state'
  // schema
  publish_json(state_topic_, [=](JsonObject root) {
    root["battery_level"] = status_.GetBatteryLevel();
    root["state"] = status_.GetState();
    root["fan_speed"] = "off";
  });
}

void RoombaComponent::wakeUp(bool initial_wake) {
  if (status_.GetSleepState()) {
    digitalWrite(brc_pin_, HIGH);
    delay(100);
    digitalWrite(brc_pin_, LOW);
    delay(500);
    digitalWrite(brc_pin_, HIGH);
    delay(100);
  }

  // I docked state roomba likes to be poked after wake up.
  // Calling dock here is harmless but it activates green button and prepares
  // Roomba for other commands
  if (status_.GetDockedState() && !initial_wake) {
    roomba_.dock();
    delay(1000);
  }
}

void RoombaComponent::on_message(const std::string& payload) {
  ESP_LOGD(TAG, "Got values %s", payload.c_str());

  wakeUp();

  // Stop previous command if in progress
  if (status_.GetCleaningState()) {
    roomba_.cover();
    delay(500);
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

// It gets list of uint8_t separated by ',', eg. playSong - 141,0
void RoombaComponent::onCustomCommand(std::string str) {
  ESP_LOGD(TAG, "onCustomCommand - %s", str.c_str());

  wakeUp();

  const auto commands = getCustomCommands(str);

  for (const auto command : commands) {
    ESP_LOGD(TAG, "sendingCustom - %d", command);
    Serial.write(command);
  }
}

RoombaComponent::RoombaComponent(const std::string& state_topic,
                                 const std::string& command_topic,
                                 uint8_t brc_pin,
                                 uint32_t update_interval)
    : PollingComponent(update_interval),
      brc_pin_(brc_pin),
      update_interval_(update_interval),
      state_topic_(state_topic),
      command_topic_(command_topic),
      roomba_(&Serial, Roomba::Baud115200),
      status_(roomba_) {}
