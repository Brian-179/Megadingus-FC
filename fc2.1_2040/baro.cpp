#include "baro.h"
#include <BME280.h>
#include <BME280I2C.h>
#include <EnvironmentCalculations.h>
#include "Arduino.h"

float barometricVelocity;
float filteredAlt;
float rawAlt;

const float referencePressure = 1013.25;// hPa local QFF (official meteor-station reading)
const float outdoorTemp = 25; // °C  measured local outdoor temp.
const float barometerAltitude = 307.0;  // meters ... map readings + barometer position, mullaley NSW is 307m
//above 3 lines to be removed

BME280I2C::Settings settings(
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::Mode_Forced,
  BME280::StandbyTime_1000ms,
  BME280::Filter_16,
  BME280::SpiEnable_False,
  BME280I2C::I2CAddr_0x76
);

BME280I2C bme(settings);                                                           

baroClass::baroClass(void){}

void baroClass::computeBaro(){
  _Alt();
  _smoothedalt();
  _barovel();
}

float baroClass::baroVel(){
  return barometricVelocity;
}

float baroClass::smoothedAlt(){
  return filteredAlt;
}

float baroClass::alt(){
  return rawAlt;
}

boolean baroClass::begin(){
  return bme.begin();
}

void _barovel() {
  static float olderMillis;
  static float oldAlt = filteredAlt;
  static float oldBaroVel = 0.0; // Initialize old velocity to zero
  //static unsigned long oldMillis = millis(); 
  float alpha = 0.1;  // Low pass filter alpha value
  float dt = (millis() - olderMillis) / 1000.0f;  // Time since last update

  float alt1 = filteredAlt;
  barometricVelocity = (alt1 - oldAlt) / dt;

  // Apply low pass filter to the velocity
  barometricVelocity = alpha * barometricVelocity + (1 - alpha) * oldBaroVel;

  oldAlt = alt1;
  oldBaroVel = barometricVelocity;
  olderMillis = millis();
}

void _smoothedalt() {
  float ALPHA = 0.03;
  float currentAlt = rawAlt;
  static float previousAlt = 0.0;

  filteredAlt = ALPHA * currentAlt + (1 - ALPHA) * previousAlt;
  previousAlt = filteredAlt;
}

void _Alt() {
  static float groundAlt;
  EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
  EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;
  float temp(NAN), hum(NAN), pres(NAN);
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa);
  bme.read(pres, temp, hum, tempUnit, presUnit);
  float altitude = EnvironmentCalculations::Altitude(pres, envAltUnit, referencePressure, outdoorTemp, envTempUnit);
  float dewPoint = EnvironmentCalculations::DewPoint(temp, hum, envTempUnit);
  float seaLevel = EnvironmentCalculations::EquivalentSeaLevelPressure(barometerAltitude, temp, pres, envAltUnit, envTempUnit);
  float absHum = EnvironmentCalculations::AbsoluteHumidity(temp, hum, envTempUnit);
  float heatIndex = EnvironmentCalculations::HeatIndex(temp, hum, envTempUnit);
  static boolean Setup = false;
  if (Setup == false){
    groundAlt = altitude;
    Setup = true;
  }
  rawAlt = altitude - groundAlt;
}
