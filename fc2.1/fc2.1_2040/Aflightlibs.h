#ifndef Aflightlibs_h
#define Aflightlibs_h

#include "Arduino.h"

class TVCClass {
  public:
    TVCClass(void);
    TVCClass(boolean _LEDEN, boolean _BUZEN);
    void estimateAngles(float opMode, float gain, float ax, float ay, float az, float gx, float gy, float gz);
    float servoAnglex();
    float servoAnglez();
    float anglex();
    float angley();
    float anglez();
    float pitch();
    float roll();
    float yaw();
    float filteredAccelY();
    float accelzSpeed();
    float accelz();
    float accelAlt();
    //float baroVel();
    //float smoothedAlt();
    //float alt();
    void pyro1();
    void pyro2();
    void pyro3();
  private:
};

void ledr(int input);
void ledg(int input);
void ledb(int input);
void exFlashSetup();
void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat);
void convertStringToNumbers(const String& inputString, float values[], int maxValues);

typedef struct Telem{
  int currMillis;
  int runMode;
  float Anglex;
  float Angley;
  float Anglez;
  float servoanglex;
  float servoanglez;
  float Alt;
  float smoothedalt;
  float vSpeed;
  float Vin;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float cycleTime;
  String lastEvent;
} Telem;

//extern TVCClass TVC;

#endif
