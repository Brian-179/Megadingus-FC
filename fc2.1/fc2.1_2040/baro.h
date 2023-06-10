#ifndef baro_h
#define baro_h

#include "Arduino.h"

class baroClass {
  public:
    baroClass(void);
    void computeBaro();
    float baroVel();
    float smoothedAlt();
    float alt();
    boolean begin();
  private:
    //    void barovel();
    //    void smoothedalt();
    //    void Alt();
};

void _barovel();
void _smoothedalt();
void _Alt(); //to be moved to private

#endif
