#pragma once

enum class Sensors {
  Distance,
  Voltage,
  Current,
  Charge,
  Capacity,
  Charging,
  BatteryLevel,
};

enum class BinarySensors {
  CleaningState,
  ChargingState,
  DockedState,
  SleepState,
};
