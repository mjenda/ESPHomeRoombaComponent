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

RoombaComponent* RoombaComponent::instance(const std::string& stateTopic,
                                           const std::string& commandTopic,
                                           uint8_t brcPin,
                                           uint32_t updateInterval) {
  static RoombaComponent* INSTANCE =
      new RoombaComponent(stateTopic, commandTopic, brcPin, updateInterval);
  return INSTANCE;
}

void RoombaComponent::setup() {
  ESP_LOGD(TAG, "Setting up roomba.");
  pinMode(this->brcPin, OUTPUT);
  digitalWrite(this->brcPin, HIGH);

  this->roomba.start();

  wakeUp();

  ESP_LOGD(TAG, "Attempting to subscribe to MQTT.");

  subscribe(this->commandTopic, &RoombaComponent::on_message);
  register_service(&RoombaComponent::onCustomCommand, "command", {"command"});
}

void RoombaComponent::update() {
  status_.OnPendingData();

  this->distanceSensor->publishState(status_.GetDistance());
  this->voltageSensor->publishState(status_.GetVoltage());
  this->currentSensor->publishState(status_.GetCurrent());
  this->chargeSensor->publishState(status_.GetCharge());
  this->capacitySensor->publishState(status_.GetCapacity());
  this->chargingBinarySensor->publishState(status_.GetCharging());
  this->dockedBinarySensor->publishState(status_.GetChargingState());
  this->cleaningBinarySensor->publishState(status_.GetCleaningState());
  this->sleepingBinarySensor->publishState(status_.GetSleepState());

  // Publish to the state topic a json document; necessary for the 'state'
  // schema
  publish_json(this->stateTopic, [=](JsonObject root) {
    root["battery_level"] = status_.GetBatteryLevel();
    root["state"] = status_.GetState();
    root["fan_speed"] = "off";
  });
}

void RoombaComponent::wakeUp() {
  if (status_.GetSleepState()) {
    digitalWrite(this->brcPin, HIGH);
    delay(100);
    digitalWrite(this->brcPin, LOW);
    delay(500);
    digitalWrite(this->brcPin, HIGH);
    delay(100);
  }

  // I docked state roomba likes to be poked after wake up.
  // Calling dock here is harmless but it activates green button and prepares
  // Roomba for other commands
  if (status_.GetDockedState()) {
    this->roomba.dock();
    delay(1000);
  }
}

void RoombaComponent::on_message(const std::string& payload) {
  ESP_LOGD(TAG, "Got values %s", payload.c_str());

  wakeUp();

  if (this->cleaningBinarySensor->state) {
    this->roomba.cover();
    delay(500);
  }

  if (payload == "turn_on" || payload == "start") {
    this->roomba.cover();
  } else if ((payload == "turn_off" || payload == "stop") &&
             !(this->cleaningBinarySensor->state)) {
    this->roomba.cover();
  } else if (payload == "dock" || payload == "return_to_base") {
    this->roomba.dock();
  } else if (payload == "locate") {
    this->roomba.playSong(1);
  } else if (payload == "spot" || payload == "clean_spot") {
    this->roomba.spot();
  } else {
    ESP_LOGW(TAG, "Received unknown status payload: %s", payload.c_str());
    this->status_momentary_warning("state", 5000);
  }

  delay(500);
  this->update();
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

RoombaComponent::RoombaComponent(const std::string& stateTopic,
                                 const std::string& commandTopic,
                                 uint8_t brcPin,
                                 uint32_t updateInterval)
    : PollingComponent(updateInterval),
      roomba(&Serial, Roomba::Baud115200),
      status_(roomba) {
  this->brcPin = brcPin;
  this->updateInterval = updateInterval;
  this->stateTopic = stateTopic;
  this->commandTopic = commandTopic;

  this->distanceSensor = new RoombaSensor<esphome::sensor::Sensor>();
  this->voltageSensor = new RoombaSensor<esphome::sensor::Sensor>();
  this->currentSensor = new RoombaSensor<esphome::sensor::Sensor>();
  this->chargeSensor = new RoombaSensor<esphome::sensor::Sensor>();
  this->capacitySensor = new RoombaSensor<esphome::sensor::Sensor>();
  this->chargingBinarySensor =
      new RoombaSensor<esphome::binary_sensor::BinarySensor>();
  this->dockedBinarySensor =
      new RoombaSensor<esphome::binary_sensor::BinarySensor>();
  this->cleaningBinarySensor =
      new RoombaSensor<esphome::binary_sensor::BinarySensor>();
  this->sleepingBinarySensor =
      new RoombaSensor<esphome::binary_sensor::BinarySensor>();
}
