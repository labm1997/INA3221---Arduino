#include <Arduino.h>
#include <Wire.h>
#include "ina3221.h"

void setup() {
  INA3221_init();
  INA3221_reset();
  pinMode(PA0, INPUT);
  pinMode(PA1, INPUT);

  //INA3221_disableChannel(INA3221_CH2);
  //INA3221_disableChannel(INA3221_CH3);
  // INA3221_setPowerValidLowerLimit(1);
  // INA3221_setPowerValidUpperLimit(1.6);
  INA3221_setChannelCriticalAlertLimit(INA3221_CH1, 0.3);
  INA3221_setChannelWarningAlertLimit(INA3221_CH1, 0.2);
}

void loop() {
  uint16_t manufacturerId = INA3221_readManufacturerId();
  float busVoltage = INA3221_readBus(INA3221_CH1);
  float shuntVoltage = INA3221_readShunt(INA3221_CH1);
  uint16_t shuntValue = readRegister(INA3221_CH1_SHUNT_VOLTAGE_REGISTER);
  uint16_t ch1CriticalAlertLimit = readRegister(INA3221_CH1_CRIT_LIMIT_REGISTER);
  uint16_t ch1WarningAlertLimit = readRegister(INA3221_CH1_WARN_LIMIT_REGISTER);
  uint16_t criticalAlert = digitalRead(PA0);
  uint16_t warningAlert = digitalRead(PA1);

  printf("%d\n", manufacturerId);
  printf("%f\n", shuntVoltage);
  printf("%d\n", shuntValue);
  printf("%d\n", criticalAlert);
  printf("%d\n", warningAlert);
  printf("%d\n", ch1CriticalAlertLimit);
  printf("%d\n", ch1WarningAlertLimit);
}