#include "ina3221.h"

static maskRegister_t maskRegister = {.ready = false};

void INA3221_init(){
    Wire.begin();
}

uint16_t readRegister(uint8_t address){
  uint8_t bytes[2];
  Wire.beginTransmission(INA3221_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(INA3221_ADDRESS, 2);
  Wire.readBytes(bytes, 2);

  return ((bytes[0]) << 8) | (bytes[1]);
}

void writeRegister(uint8_t address, uint16_t value){
  Wire.beginTransmission(INA3221_ADDRESS);
  Wire.write(address);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0x00ff));
  Wire.endTransmission();
}

void setRegisterBit(uint8_t address, uint8_t bit){
  uint16_t registerValue = readRegister(address);
  writeRegister(address, registerValue | (1 << bit));
}

void unsetRegisterBit(uint8_t address, uint8_t bit){
  uint16_t registerValue = readRegister(address);
  writeRegister(address, registerValue & ~(1 << bit));
}

void INA3221_reset(){
  writeRegister(INA3221_CONFIG_REGISTER, 0x8000);
}

void INA3221_enableChannel(INA3221_CHANNELS channel){
  setRegisterBit(INA3221_CONFIG_REGISTER, 14 - channel + 1);
}

void INA3221_disableChannel(INA3221_CHANNELS channel){
  unsetRegisterBit(INA3221_CONFIG_REGISTER, 14 - channel + 1);
}

void INA3221_setAveragingMode(INA3221_AVERAGING_MODE mode){
  uint16_t maskedConfigRegister = readRegister(INA3221_CONFIG_REGISTER) & 0b1111000111111111;
  writeRegister(INA3221_CONFIG_REGISTER, maskedConfigRegister | (mode << 9));
}

void INA3221_setBusConversionTime(INA3221_CONVERSION_TIME conversionTime){
  uint16_t maskedConfigRegister = readRegister(INA3221_CONFIG_REGISTER) & 0b1111111000111111;
  writeRegister(INA3221_CONFIG_REGISTER, maskedConfigRegister | (conversionTime << 6));
}

void INA3221_setMode(INA3221_OPERATING_MODE mode){
  uint16_t maskedConfigRegister = readRegister(INA3221_CONFIG_REGISTER) & 0b1111111111111000;
  writeRegister(INA3221_CONFIG_REGISTER, maskedConfigRegister | (mode << 0));
}

uint8_t channelToBusRegister(INA3221_CHANNELS channel){
  switch(channel){
    case INA3221_CH1:
      return INA3221_CH1_BUS_VOLTAGE_REGISTER;
    case INA3221_CH2:
      return INA3221_CH2_BUS_VOLTAGE_REGISTER;
    case INA3221_CH3:
      return INA3221_CH3_BUS_VOLTAGE_REGISTER;
  }
  return -1;
}

uint8_t channelToShuntRegister(INA3221_CHANNELS channel){
  switch(channel){
    case INA3221_CH1:
      return INA3221_CH1_SHUNT_VOLTAGE_REGISTER;
    case INA3221_CH2:
      return INA3221_CH2_SHUNT_VOLTAGE_REGISTER;
    case INA3221_CH3:
      return INA3221_CH3_SHUNT_VOLTAGE_REGISTER;
  }
  return -1;
}

uint8_t channelToCriticalLimitRegister(INA3221_CHANNELS channel){
  switch(channel){
    case INA3221_CH1:
      return INA3221_CH1_CRIT_LIMIT_REGISTER;
    case INA3221_CH2:
      return INA3221_CH2_CRIT_LIMIT_REGISTER;
    case INA3221_CH3:
      return INA3221_CH3_CRIT_LIMIT_REGISTER;
  }
  return -1;
}

uint8_t channelToWarningLimitRegister(INA3221_CHANNELS channel){
  switch(channel){
    case INA3221_CH1:
      return INA3221_CH1_WARN_LIMIT_REGISTER;
    case INA3221_CH2:
      return INA3221_CH2_WARN_LIMIT_REGISTER;
    case INA3221_CH3:
      return INA3221_CH3_WARN_LIMIT_REGISTER;
  }
  return -1;
}

/*
  Returns shunt voltage in mV
*/
float valueToShuntVoltage(int16_t value){
  return (value >> 3) * 40 / 1000.0;
}

/*
  @param shuntVoltage in mV
*/
int16_t shuntVoltageToValue(float shuntVoltage){
  return (int16_t)(shuntVoltage * 1000 / 40) << 3;
}

/*
  Returns bus voltage in V
*/
float valueToBusVoltage(int16_t value){
  return (value >> 3) * 8 / 1000.0;
}

/*
  @param busVoltage in V
*/
int16_t busVoltageToValue(float busVoltage){
  return (int16_t)(busVoltage * 1000 / 8) << 3;
}

/*
  Returns voltage in mV
*/
float INA3221_readShunt(INA3221_CHANNELS channel){
  int16_t shuntValue = ((int16_t)readRegister(channelToShuntRegister(channel)));
  return valueToShuntVoltage(shuntValue);
}

/*
  Returns voltage in V
*/
float INA3221_readBus(INA3221_CHANNELS channel){
  int16_t busValue = ((int16_t)readRegister(channelToBusRegister(channel)));
  return valueToBusVoltage(busValue);
}

/*
  shuntVoltage in mV
*/
void INA3221_setChannelCriticalAlertLimit(INA3221_CHANNELS channels, float shuntVoltage){
  writeRegister(channelToCriticalLimitRegister(channels), shuntVoltageToValue(shuntVoltage));
}

/*
  shuntVoltage in mV
*/
void INA3221_setChannelWarningAlertLimit(INA3221_CHANNELS channels, float shuntVoltage){
  writeRegister(channelToWarningLimitRegister(channels), shuntVoltageToValue(shuntVoltage));
}

/*
  Returns sum of all sum enabled shuntVoltage in mV. Can be used to compute total amount of current flowing through INA3221
*/
float INA3221_readShuntSum(){
  return (((int16_t)readRegister(INA3221_SHUNT_VOLTAGE_SUM_REGISTER)) >> 1) * 40 / 1000.0;
}

/*
  @param sumVoltage in mV
*/
void INA3221_setReadShuntSumLimit(float sumVoltage){
  uint16_t sumValue = (uint16_t)((int16_t)(sumVoltage * 1000 / 40) << 1);
  writeRegister(INA3221_SHUNT_VOLTAGE_SUM_LIMIT_REGISTER, sumValue);
}

void INA3221_updateMaskRegister(){
  uint16_t fetchedMaskRegister = readRegister(INA3221_MASK_ENABLE_REGISTER);

  maskRegister.conversionReadyFlag       = INA3221_READBIT(fetchedMaskRegister, 0);
  maskRegister.timingControlAlertFlag    = INA3221_READBIT(fetchedMaskRegister, 1);
  maskRegister.powerValidAlertFlag       = INA3221_READBIT(fetchedMaskRegister, 2);
  maskRegister.ch1WarningAlertFlag       = INA3221_READBIT(fetchedMaskRegister, 3);
  maskRegister.ch2WarningAlertFlag       = INA3221_READBIT(fetchedMaskRegister, 4);
  maskRegister.ch3WarningAlertFlag       = INA3221_READBIT(fetchedMaskRegister, 5);
  maskRegister.summationAlertFlag        = INA3221_READBIT(fetchedMaskRegister, 6);
  maskRegister.ch1CriticalAlertFlag      = INA3221_READBIT(fetchedMaskRegister, 7);
  maskRegister.ch2CriticalAlertFlag      = INA3221_READBIT(fetchedMaskRegister, 8);
  maskRegister.ch3CriticalAlertFlag      = INA3221_READBIT(fetchedMaskRegister, 9);
  maskRegister.criticalAlertLatchEnable  = INA3221_READBIT(fetchedMaskRegister, 10);
  maskRegister.warningAlertLatchEnable   = INA3221_READBIT(fetchedMaskRegister, 11);
  maskRegister.ch1SummationControlEnable = INA3221_READBIT(fetchedMaskRegister, 12);
  maskRegister.ch2SummationControlEnable = INA3221_READBIT(fetchedMaskRegister, 13);
  maskRegister.ch3SummationControlEnable = INA3221_READBIT(fetchedMaskRegister, 14);

  maskRegister.ready = true;
}

/*
 * @warning This will erase Critical-alert flag, Summation-alert flag, Warning-alert flag
 */
void INA3221_enableChannelSumMode(INA3221_CHANNELS channel){
  setRegisterBit(INA3221_MASK_ENABLE_REGISTER, 14 - channel + 1);
}

/*
 * @warning This will erase Critical-alert flag, Summation-alert flag, Warning-alert flag
 */
void INA3221_disableChannelSumMode(INA3221_CHANNELS channel){
  unsetRegisterBit(INA3221_MASK_ENABLE_REGISTER, 14 - channel + 1);
}

/*
 * @warning This will erase Critical-alert flag, Summation-alert flag, Warning-alert flag
 */
void INA3221_enableWarningAlertLatch(){
  setRegisterBit(INA3221_MASK_ENABLE_REGISTER, 11);
}

/*
 * @warning This will erase Critical-alert flag, Summation-alert flag, Warning-alert flag
 */
void INA3221_disableWarningAlertLatch(){
  unsetRegisterBit(INA3221_MASK_ENABLE_REGISTER, 11);
}

/*
 * @warning This will erase Critical-alert flag, Summation-alert flag, Warning-alert flag
 */
void INA3221_enableCriticalAlertLatch(){
  setRegisterBit(INA3221_MASK_ENABLE_REGISTER, 10);
}

/*
 * @warning This will erase Critical-alert flag, Summation-alert flag, Warning-alert flag
 */
void INA3221_disableCriticalAlertLatch(){
  unsetRegisterBit(INA3221_MASK_ENABLE_REGISTER, 10);
}

/*
 * @warning To get updated data run `updateMaskRegister` first, however, running `updateMaskRegister` can erase Critical-alert flag, Summation-alert flag, Warning-alert flag. So run it once and get everything you need immediately after.
 * @return if readings are valid
 */
bool INA3221_getCriticalChannelsFlag(bool *ch1, bool *ch2, bool *ch3){
  *ch1 = maskRegister.ch1CriticalAlertFlag;
  *ch2 = maskRegister.ch2CriticalAlertFlag;
  *ch3 = maskRegister.ch3CriticalAlertFlag;

  return maskRegister.ready;
}

/*
 * @warning To get updated data run `updateMaskRegister` first, however, running `updateMaskRegister` can erase Critical-alert flag, Summation-alert flag, Warning-alert flag. So run it once and get everything you need immediately after.
 * 
 * @return if readings are valid
 */
bool INA3221_getSummationAlertFlag(bool *summationAlertFlag){
  *summationAlertFlag = maskRegister.summationAlertFlag;

  return maskRegister.ready;
}

/*
 * @warning To get updated data run `updateMaskRegister` first, however, running `updateMaskRegister` can erase Critical-alert flag, Summation-alert flag, Warning-alert flag. So run it once and get everything you need immediately after.
 * 
 * @return if readings are valid
 */
bool INA3221_getCWarningChannelsFlag(bool *ch1, bool *ch2, bool *ch3){
  *ch1 = maskRegister.ch1WarningAlertFlag;
  *ch2 = maskRegister.ch2WarningAlertFlag;
  *ch3 = maskRegister.ch3WarningAlertFlag;

  return maskRegister.ready;
}

/*
 * @warning To get updated data run `updateMaskRegister` first, however, running `updateMaskRegister` can erase Critical-alert flag, Summation-alert flag, Warning-alert flag. So run it once and get everything you need immediately after.
 * 
 * @return if readings are valid
 */
bool INA3221_getPowerValidAlertFlag(bool *powerValidAlertFlag){
  *powerValidAlertFlag = maskRegister.powerValidAlertFlag;

  return maskRegister.ready;
}

/*
 * @warning To get updated data run `updateMaskRegister` first, however, running `updateMaskRegister` can erase Critical-alert flag, Summation-alert flag, Warning-alert flag. So run it once and get everything you need immediately after.
 * 
 * @return if readings are valid
 */
bool INA3221_getTimingControlAlertFlag(bool *timingControlAlertFlag){
  *timingControlAlertFlag = maskRegister.timingControlAlertFlag;

  return maskRegister.ready;
}

/*
 * @warning To get updated data run `updateMaskRegister` first, however, running `updateMaskRegister` can erase Critical-alert flag, Summation-alert flag, Warning-alert flag. So run it once and get everything you need immediately after.
 * 
 * @return if readings are valid
 */
bool INA3221_getConversionReadyFlag(bool *conversionReadyFlag){
  *conversionReadyFlag = maskRegister.conversionReadyFlag;

  return maskRegister.ready;
}

/*
  @param busVoltage in V
*/
void INA3221_setPowerValidUpperLimit(float busVoltage){
  uint16_t busValue = (uint16_t)busVoltageToValue(busVoltage);
  writeRegister(INA3221_POWER_VALID_UPPER_LIMIT_REGISTER, busValue);
}

/*
  @param busVoltage in V
*/
void INA3221_setPowerValidLowerLimit(float busVoltage){
  uint16_t busValue = (uint16_t)busVoltageToValue(busVoltage);
  writeRegister(INA3221_POWER_VALID_LOWER_LIMIT_REGISTER, busValue);
}

uint16_t INA3221_readManufacturerId(){
  return readRegister(INA3221_MANUFACTURER_ID_REGISTER);
}

uint16_t INA3221_readDieId(){
  return readRegister(INA3221_DIE_ID_REGISTER);
}