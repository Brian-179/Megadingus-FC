#ifndef Aflightlibs_h
#define Aflightlibs_h

#include "Arduino.h"

class TVCClass {
  public:
    TVCClass(void);
    void estimateAngles(float opMode, float gain, float ax, float ay, float az, float gx, float gy, float gz);
    float anglex();
    float angley();
    float anglez();
  private:
};

void ledr(int input);
void ledg(int input);
void ledb(int input);
void exFlashSetup();

//extern TVCClass TVC;

#endif
