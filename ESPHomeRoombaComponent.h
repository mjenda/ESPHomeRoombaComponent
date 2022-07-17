#include "esphome.h"
#include <Roomba.h>

static const char *TAG = "component.Roomba";
class RoombaComponent : public PollingComponent, public CustomMQTTDevice
{
  protected:
    uint8_t brcPin;
    uint32_t updateInterval;
    std::string stateTopic;
    std::string commandTopic;
    Roomba roomba;

  public:
    Sensor *distanceSensor;
    Sensor *voltageSensor;
    Sensor *currentSensor;
    Sensor *chargeSensor;
    Sensor *capacitySensor;
    BinarySensor *chargingBinarySensor;
    BinarySensor *dockedBinarySensor;
    BinarySensor *cleaningBinarySensor;

    static RoombaComponent* instance(const std::string &stateTopic, const std::string &commandTopic, uint8_t brcPin, uint32_t updateInterval)
    {
        static RoombaComponent* INSTANCE = new RoombaComponent(stateTopic, commandTopic, brcPin, updateInterval);
        return INSTANCE;
    }

    void setup() override
    {
        ESP_LOGD(TAG, "Setting up roomba.");
        pinMode(this->brcPin, OUTPUT);
        digitalWrite(this->brcPin, HIGH);

        this->roomba.start();

        ESP_LOGD(TAG, "Attempting to subscribe to MQTT.");

        subscribe(this->commandTopic, &RoombaComponent::on_message);
    }

    void update() override
    {
        ESP_LOGD(TAG, "Attempting to update sensor values.");

        int16_t distance;
        uint16_t voltage;
        int16_t current;
        uint16_t charge;
        uint16_t capacity;
        uint8_t charging;
        bool cleaningState;
        bool dockedState;
        bool chargingState;
        bool publishJson;
        // Flush serial buffers
        while (Serial.available())
        {
            Serial.read();
        }

        uint8_t sensors[] = {
            Roomba::SensorDistance,       // 2 bytes, mm, signed
            Roomba::SensorChargingState,  // 1 byte
            Roomba::SensorVoltage,        // 2 bytes, mV, unsigned
            Roomba::SensorCurrent,        // 2 bytes, mA, signed
            Roomba::SensorBatteryCharge,  // 2 bytes, mAh, unsigned
            Roomba::SensorBatteryCapacity // 2 bytes, mAh, unsigned
        };
        uint8_t values[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        // Serial reading timeout -- https://community.home-assistant.io/t/add-wifi-to-an-older-roomba/23282/52
        bool success = this->roomba.getSensorsList(sensors, sizeof(sensors), values, sizeof(values));
        if (!success)
        {
            ESP_LOGD(TAG, "Unable to read sensors from the roomba.");
            return;
        }

        distance = values[0] * 256 + values[1];
        voltage = values[3] * 256 + values[4];
        current = values[5] * 256 + values[6];
        charge = values[7] * 256 + values[8];
        capacity = values[9] * 256 + values[10];
        charging = values[2];

        cleaningState = current < -300;
        dockedState = current > -50;
        chargingState = charging == Roomba::ChargeStateReconditioningCharging || charging == Roomba::ChargeStateFullChanrging || charging == Roomba::ChargeStateTrickleCharging;

        // Only publish new states if there was a change
        if (this->distanceSensor->state != distance) {
            this->distanceSensor->publish_state(distance);
        }

        if (this->voltageSensor->state != voltage) {
            this->voltageSensor->publish_state(voltage);
        }

        if (this->currentSensor->state != current) {
            this->currentSensor->publish_state(current);
        }

        if (this->chargeSensor->state != charge) {
            this->chargeSensor->publish_state(charge);
        }

        if (this->capacitySensor->state != capacity) {
            this->capacitySensor->publish_state(capacity);
        }

        if (this->chargingBinarySensor->state != chargingState) {
            this->chargingBinarySensor->publish_state(chargingState);
        }

        if (this->dockedBinarySensor->state != dockedState) {
            this->dockedBinarySensor->publish_state(dockedState);
        }

        if (this->cleaningBinarySensor->state != cleaningState) {
            this->cleaningBinarySensor->publish_state(cleaningState);
        } 

        static std::string lastBatteryLevel = "0.0";
        static std::string lastState;
        std::string batteryLevel = value_accuracy_to_string(100.0 * ((1.0 * charge) / (1.0 * capacity)), 2);
        std::string state = cleaningState ? "cleaning" : 
                            dockedState ? "docked" :
                            chargingState ? "idle" :
                            "idle";

        // Publish to the state topic a json document; necessary for the 'state' schema
        if (batteryLevel != lastBatteryLevel || state != lastState) {
            lastBatteryLevel = batteryLevel;
            lastState = state;

            publish_json(this->stateTopic, [=](JsonObject root) {
                root["battery_level"] = parse_float(batteryLevel).value();
                root["state"] = state;
                root["fan_speed"] = "off";
            });
        }
    }

    void on_message(const std::string &payload) 
    {
        ESP_LOGD(TAG, "Got values %s", payload.c_str());

        // Roomba Wakeup
        digitalWrite(this->brcPin, HIGH);
        delay(100);
        digitalWrite(this->brcPin, LOW);
        delay(500);
        digitalWrite(this->brcPin, HIGH);
        delay(100);

        if (payload == "turn_on" || payload == "turn_off" || 
            payload == "start" || payload == "stop")
        {
            this->roomba.cover();
        }
        else if (payload == "dock" || payload == "return_to_base")
        {
            this->roomba.dock();
        }
        else if (payload == "locate")
        {
            this->roomba.playSong(1);
        }
        else if (payload == "spot" || payload == "clean_spot")
        {
            this->roomba.spot();
        }
        else
        {
            ESP_LOGW(TAG, "Received unknown status payload: %s", payload.c_str());
            this->status_momentary_warning("state", 5000);
        }

        delay(500);
        this->update();
    }

  private: 
    RoombaComponent(const std::string &stateTopic, const std::string &commandTopic, uint8_t brcPin, uint32_t updateInterval) : 
        PollingComponent(updateInterval), roomba(&Serial, Roomba::Baud115200)
    {
        this->brcPin = brcPin;
        this->updateInterval = updateInterval;
        this->stateTopic = stateTopic;
        this->commandTopic = commandTopic;
        
        this->distanceSensor = new Sensor();
        this->voltageSensor = new Sensor();
        this->currentSensor = new Sensor();
        this->chargeSensor = new Sensor();
        this->capacitySensor = new Sensor();
        this->chargingBinarySensor = new BinarySensor();
        this->dockedBinarySensor = new BinarySensor();
        this->cleaningBinarySensor = new BinarySensor();
    }
};
