#include <EEPROM.h>
//#include <SD.h>
//#include <SPIMemory.h>
//#include <PID_v1.h>
#include <SPI.h>
#include <EnvironmentCalculations.h>
#include <BME280I2C.h>
#include <Servo.h>
#include "BMI088.h" 

float servoLimit = 10; //servo actuation limit on either side e.g. 10deg = 10deg on either side = 20deg total
float centrepoint = 0; //servo centrepoint,usually 90

double Kp = 0.1;  // PID loop values
double Ki = 0;
double Kd = 0.033;
//double Setpoint = 90;

float targetAnglex = 0;
float targetAnglez = 0;

float delayPeriod = 3; //backup delay period if no baro (from burnout to ejection)

float gyroThreshold = 0.1; //threshold for speed of rotation to detect landing

float gravity = 9.73; //gravity
float offset1;

float referencePressure = 1018.6;  // hPa local QFF (official meteor-station reading)
float outdoorTemp = 25;           // °C  measured local outdoor temp.
float barometerAltitude = 1650.3;  // meters ... map readings + barometer position

double anglex;
double angley;
double anglez;
//double servoAnglex;
//double servoAnglez;
double anglex1;
double anglez1;
float accelSpeedy;
float accelAlt;
float oldMillis2;
float oldMillis3;
float cycleTime;
int runMode = 0;
int altcount = 0;
float altitude;
unsigned long currMillis;
unsigned long oldMillis;
const int SDCS = PB1;
boolean SDCard = true;

Servo servox;
Servo servoy;

//File file1;

//PID pidx(&anglex, &anglex1, &Setpoint, Kp, Ki, Kd, DIRECT);
//PID pidz(&anglez, &anglez1, &Setpoint, Kp, Ki, Kd, DIRECT);

Bmi088Accel accel(Wire,0x18);
Bmi088Gyro gyro(Wire,0x68);

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
//SPIFlash flash;
//Sd2Card card;
//SdVolume volume;
//SdFile piss;

void setup() {
  int status;
  status = accel.begin();
  status = gyro.begin();
  Serial.begin(115200);
  Serial.print("angle test");
  Wire.begin();

  oldMillis2 = 5;
  oldMillis3 = 5;
  
  //pidx.SetMode(AUTOMATIC);
  //pidz.SetMode(AUTOMATIC);

  pinMode(PB0,OUTPUT); //pyro1
  //file1 = SD.open("piss.txt", FILE_WRITE);
  
  servox.attach(PD5);
  servoy.attach(PD6);
  servox.write(90);
  servoy.write(90);
    if (!gyro.begin()){
    Serial.println("no gyro");
    while(1);
  }

  while (!bme.begin()){
    Serial.println("no baro");
    while(1);
  }
  
//  if (!SD.begin(PB1)){
//    Serial.println("No SD card!");
//    SDCard = false;
//  }

  //gravity = accel.getAccelY_mss();
  
  delay(200);
  servox.write(80);
  delay(100);
  servox.write(90);
  delay(100);
  servox.write(100);
  delay(100);
  servox.write(90);
  delay(100);
  servoy.write(80);
  delay(100);
  servoy.write(90);
  delay(100);
  servoy.write(100);
  delay(100);
  servoy.write(90);
}

void loop() {
  currMillis = millis();
  cycleTime = (currMillis - oldMillis2);
  accel.readSensor();
  gyro.readSensor();

  accelSpeedy = (((accelSpeedy + (accel.getAccelY_mss())) - gravity));
  accelAlt = (accelAlt + accelSpeedy);
    
//  anglex = (anglex + (gyro.getGyroX_rads() * (M_PI / 180) * ((4078/71) * 1.5)));
//  angley = (angley + (gyro.getGyroY_rads() * (M_PI / 180) * ((4078/71) * 1.5)));
//  anglez = (anglez + (gyro.getGyroZ_rads() * (M_PI / 180) * ((4078/71) * 1.5)));

  anglex = (anglex + (gyro.getGyroX_rads() * (M_PI / 180) * ((4078/71) * (cycleTime / 10))));
  angley = (angley + (gyro.getGyroY_rads() * (M_PI / 180) * ((4078/71) * (cycleTime / 10))));
  anglez = (anglez + (gyro.getGyroZ_rads() * (M_PI / 180) * ((4078/71) * (cycleTime / 10))));

//  anglex = (((anglex + (gyro.getGyroX_rads())) * 4068) /71);
//  angley = (((anglex + (gyro.getGyroX_rads())) * 4068) /71);
//  anglez = (((anglex + (gyro.getGyroX_rads())) * 4068) /71);
  
  if (runMode == 0){
    wait();
  }
  if (runMode == 1){
    poweredAscent();
  }
  if (runMode == 2){
    coast();
  }
  if (runMode == 3){
    descent();
  }
  if (runMode == 4){
    landed();
  }
  
//  Serial.println(alt());

  Serial.print(anglex);
  Serial.print("     ");
  Serial.print(angley);
  Serial.print("     ");
  Serial.print(anglez);
  Serial.print("     ");
  //Serial.print(alt());
  //Serial.print("     ");
  //Serial.print(cycleTime);
  //Serial.print("     ");
  //Serial.print(accelSpeedy);
  //Serial.print("     ");
  //Serial.print(accelAlt);
  Serial.print(servoAnglex());
  Serial.print("     ");
  Serial.print(servoAnglez());
  Serial.print("     ");

//  if (SDCard == true){
//    file1.print(anglex);
//    file1.print("     ");
//    file1.print(anglez);
//    file1.print("     ");
//    file1.print(servoAnglex());
//    file1.print("     ");
//    file1.println(servoAnglez());
//  }

//  Serial.print(accel.getAccelX_mss());
//  Serial.print("     ");
//  Serial.print(accel.getAccelY_mss());
//  Serial.print("     ");
//  Serial.println(accel.getAccelZ_mss());

 //Y axis is up/down
 
  //Serial.println(gyro.getGyroX_rads());
  //pidx.Compute();
  //pidz.Compute();
  
  //servoAnglex = anglex1 + 70;
  //servoAnglez = anglez1 + 70;
  oldMillis2 = currMillis;
  oldMillis3 = currMillis;
  Serial.println();
  delay(10);
}

void wait(){
  boolean foundGravity = false;
  if (foundGravity == false){
    offset1 = (accel.getAccelY_mss() - gravity);
    foundGravity = true;
  }
  Serial.println("waiting on pad");
  anglex = 0;
  angley = 0;
  anglez = 0; // prevent gyro drift while waiting on pad
  //servoAnglex = 90;
  //servoAnglez = 90;
  accelSpeedy = 0;

  if (accel.getAccelY_mss() >= 11) {
    Serial.println("powered ascent");
    runMode = 1;
  }
}

void poweredAscent() {
  servox.write(servoAnglex);
  servoy.write(servoAnglez);

  if (accel.getAccelY_mss() <= 7) {
    servox.write(90);
    servoy.write(90); //homes and locks servos
    Serial.println("coasting");
    oldMillis = currMillis;
    runMode = 2;
  }
}

void coast() {
  float altitude2 = alt();
  altcount += 1;
  //Serial.println(altcount);
  //delay(500);
  //if (altcount >=delayPeriod / 2)
  if ((alt() <= (altitude2 - 0.05)) or ((currMillis - oldMillis) >= (delayPeriod * 1000))){
    Serial.println("descending");
    oldMillis = currMillis;
    runMode = 3;
  }
}

void descent() {
  digitalWrite(PB0,HIGH);//pyro charge 1
  if ((gyro.getGyroX_rads() <= gyroThreshold) & (gyro.getGyroY_rads() <= gyroThreshold) & (gyro.getGyroZ_rads() <= gyroThreshold) & (gyro.getGyroX_rads() >= -gyroThreshold) & (gyro.getGyroY_rads() >= -gyroThreshold) & (gyro.getGyroZ_rads() >= -gyroThreshold)) {
    Serial.println("landed!");
    runMode = 4;
  }
}

void landed() {
}

//custom libraries

float alt(){
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
   return altitude;
}

float servoAnglex(){
  float error;
  float cumulative_error;
  float prev_error;
  float pval;
  float ival;
  float dval;
  float pidval;
  error = targetAnglex - anglex;
  cumulative_error += error;
  pval = error * Kp;
  ival = cumulative_error * Ki * (currMillis - oldMillis2);
  dval = ((error - prev_error) / (currMillis - oldMillis2)) * Kd;
  pidval = (pval + ival + dval + centrepoint);
  if (pidval >= servoLimit + centrepoint){
    pidval = servoLimit + centrepoint;
  }
  if (pidval <= -servoLimit + centrepoint){
    pidval = -servoLimit + centrepoint;
  }
  prev_error = error;
  //Serial.print("     ");
  //Serial.print(currMillis - oldMillis2);
  //oldMillis2 = currMillis;
  return pidval;
}

float servoAnglez(){
  float error;
  float cumulative_error;
  float prev_error;
  float pval;
  float ival;
  float dval;
  float pidval;
  error = targetAnglez - anglez;
  cumulative_error += error;
  pval = error * Kp;
  ival = cumulative_error * Ki * (currMillis - oldMillis3);
  dval = (error - prev_error) / (currMillis - oldMillis3) * Kd;
  pidval = (pval + ival + dval + centrepoint);
  if (pidval >= servoLimit + centrepoint){
    pidval = servoLimit + centrepoint;
  }
  if (pidval <= -servoLimit + centrepoint){
    pidval = -servoLimit + centrepoint;
  }
  prev_error = error;
  //Serial.print("     ");
  //Serial.println(currMillis - oldMillis3);
  //oldMillis3 = currMillis;
  return pidval;
}
