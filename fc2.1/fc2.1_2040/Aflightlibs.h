#ifndef Aflightlibs_h
#define Aflightlibs_h

#include "Arduino.h"

class TVCClass {
  public:
    TVCClass(void);
    TVCClass(boolean _LEDEN, boolean _BUZEN);
    void estimateAngles(float opMode, float gain, float ax, float ay, float az, float gx, float gy, float gz);
    float anglex();
    float angley();
    float anglez();
    float pitch();
    float roll();
    float yaw();
    float filteredAccelY();
    void pyro1();
    void pyro2();
    void pyro3();
  private:
};

void ledr(int input);
void ledg(int input);
void ledb(int input);
void exFlashSetup();

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
