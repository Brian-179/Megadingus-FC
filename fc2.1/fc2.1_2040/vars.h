#include <Servo.h>

int pyro1 = 10;
int pyro2 = 11;
int pyro3 = 12;
int addr;
int readAddr = 0;
int runMode = 0; 
double Kp;  // PID loop values
double Ki;
double Kd;
float delayPeriod; //backup delay period if no baro (from burnout to ejection)
float datalogInterval = 1000; //datalogging rate
unsigned long currMillis;
unsigned long oldMillis;
float oldMillis6; //datalog stuff
float oldMillis2;
float oldMillis3;
float oldMillis4 = 0;
float cycleTime;
boolean statWrite;
int startAddr = 12017; //EEPROM writing start address for datalogging.

//double Setpoint = 90;

float targetAnglex = 0;
float targetAnglez = 0;

float gyroThreshold = 0.01; //threshold for speed of rotation to detect landing

float gravity = 9.73; //gravity
float offset1;

//float referencePressure = 1018.6;  // hPa local QFF (official meteor-station reading)
//float outdoorTemp = 25;           // °C  measured local outdoor temp.
//float barometerAltitude = 1650.3;  // meters ... map readings + barometer position

double anglex;
double angley;
double anglez;
//float roll, pitch, yaw;
//double servoAnglex;
//double servoAnglez;
double anglex1;
double anglez1;
float accelSpeedy;
float accelAlt;
float groundAlt = 0;
//float vin;
int altcount = 0;
float altitude;
boolean SDCard = true;
boolean led = false;
boolean datalogging;
boolean lockGyro = false;
boolean booted = false;
float vSpeed;
boolean seenSerialInput;
boolean SDAvailable = true;
String serialInput;

float vspeed;
float servoanglex;
float servoanglez;
float Alt;
float smoothedalt;
float ax, ay, az;
float gx, gy, gz;
float Vin;
float flightTime;
float ignitionTime;

String datalogString;
Telem telem;
String lastEvent;

Servo servox;
Servo servoy;
Servo prop;
Servo parachuteServo;
