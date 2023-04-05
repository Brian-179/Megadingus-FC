#include "Aflightlibs.h"
#include "Arduino.h"
#include <SerialFlash.h>

//#define GET_FIELD(regname,value) ((value & regname##_MASK) >> regname##_POS)
//#define  SET_FIELD(regval,regname,value) ((regval & ~regname##_MASK) | ((value << regname##_POS) & regname##_MASK))

float x, y, z;

//TVCClass::TVCClass(void){}

TVCClass::TVCClass(){
  
}

float TVCClass::anglex(){
  return x;
}

float TVCClass::angley(){
  return y;
}

float TVCClass::anglez(){
  return z;
}

void TVCClass::estimateAngles(float opMode, float gain, float ax, float ay, float az, float gx, float gy, float gz) {
  float Kd = 0;
  //rp2040.idleOtherCore();
  float xerror;
  float zerror;
  static float xprev_error;
  static float zprev_error;
  static float xcumulative_error;
  static float zcumulative_error;
  float accelAnglex;
  float accelAnglez;
  float xival;
  float xdval;
  float zival;
  float zdval;
  static int olderMillis;
  int dt = millis() - olderMillis;
  accelAnglex = -(atan2(az, sqrt(pow(ax, 2) + pow(ay, 2))) * 180 / PI);
  accelAnglez = (atan2(ax, sqrt(pow(ay, 2) + pow(az, 2))) * 180 / M_PI);
  x += (gx * (M_PI / 180) * ((4078 / 71) * (dt / 10) * 0.73));  //0.6
  y += (gy * (M_PI / 180) * ((4078 / 71) * (dt / 10) * 0.73));
  z += (gz * (M_PI / 180) * ((4078 / 71) * (dt / 10) * 0.73));
  if (opMode == 0){
    x = accelAnglex;
    y = 0;
    z = accelAnglez;
  }
  
//  xerror = accelAnglex - x;
//  xcumulative_error += xerror * dt;
//  xival = (xcumulative_error * gain) / 10000;
//  xdval = ((xerror - xprev_error) / dt) * Kd;
//  x += xival;
//
//  zerror = accelAnglez - z;
//  zcumulative_error += zerror * dt;
//  zival = (zcumulative_error * gain) / 10000;
//  xdval = ((zerror - zprev_error) / dt) * Kd;
//  z += zival;
  
//  if (x <= accelAnglex){
//    x += (gain * (dt / 10));
//  }
//  if (x >= accelAnglex){
//    x -= (gain * (dt / 10));
//  }
//  if (z <= accelAnglez){
//    z += (gain * (dt / 10));
//  }
//  if (z >= accelAnglez){
//    z -= (gain * (dt / 10));
//  }
  //Serial.println(dt);
  xprev_error = xerror;
  zprev_error = zerror;
  olderMillis = millis();
}

void ledr(int input) {
  if (input == 1) {
    digitalWrite(18, LOW);
  }
  if (input == 0) {
    digitalWrite(18, HIGH);
  }
  //  else if ((input >= 0) & (input <= 255)){
  //    analogWrite(18, 255 - input);
  //  }
}

void ledg(int input) {
  if (input == 1) {
    digitalWrite(19, LOW);
  }
  if (input == 0) {
    digitalWrite(19, HIGH);
  }
  //  else if ((input >= 0) & (input <= 255)){
  //    analogWrite(19, 255 - input);
  //  }
}

void ledb(int input) {
  if (input == 1) {
    digitalWrite(20, LOW);
  }
  if (input == 0) {
    digitalWrite(20, HIGH);
  }
  //  else if ((input >= 0) & (input <= 255)){
  //    analogWrite(20, 255 - input);
  //  }
}

void buz(int input) {
  if (input == 1 ) {
    tone(9, 200, 5);
  }
  else {

  }
}

//TVCClass TVC;
