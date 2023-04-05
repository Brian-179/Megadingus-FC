#include <SPI.h>

int espcs;

class SPITelem {
  public:
    //SPITelem(void);
    SPITelem(int espcs) {
      //      SPI.setRX(4);
      //      SPI.setTX(3);
      //      SPI.setSCK(2);
//      if (!SPI.begin){
//        SPI.begin();
//      }
      pinMode(espcs, OUTPUT);
      digitalWrite(espcs, HIGH);
    }
    //    void begin(int espcs) {
    //      SPI.setRX(4);
    //      SPI.setTX(3);
    //      SPI.setSCK(2);
    //      SPI.begin();
    //      pinMode(espcs, OUTPUT);
    //      digitalWrite(espcs, HIGH);
    //    }
    void send(int input) {
      digitalWrite(espcs, LOW);
      for (int i = 0; i < sizeof(input); i++) {
        SPI.transfer(input);
      }
      digitalWrite(espcs, HIGH);
    }
    //    void send() {
    //      digitalWrite(_SPITelemCS, LOW);
    //      SPI.transfer(1);
    //      SPI.transfer(currMillis);
    //      SPI.transfer(runMode);
    //      SPI.transfer(anglex);
    //      SPI.transfer(angley);
    //      SPI.transfer(anglez);
    //      SPI.transfer(servoanglex - centrepoint);
    //      SPI.transfer(servoanglez - centrepoint);
    //      SPI.transfer(Alt);
    //      SPI.transfer(smoothedalt);
    //      SPI.transfer(vSpeed);
    //      SPI.transfer(Vin);
    //      SPI.transfer(ax);
    //      SPI.transfer(ay);
    //      SPI.transfer(az);
    //      digitalWrite(_SPITelemCS, HIGH);
    //    }
};
