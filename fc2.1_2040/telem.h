#include <SPI.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include "Aflightlibs.h"
#ifndef telem_h
#define telem_h

class SPITelem {
  public:
    //SPITelem(void);
    SPITelem(int espcs);
    void send(String input);
    void send(float input);
    void send(Telem input);
    String receive();
    String read();
  private:
};

class SoftwareSerialOneWire{
  public:
    SoftwareSerialOneWire(int serialpin, int Baud);
    boolean available();
    void println(String sinput);
    String read();
    String waitRead();
    String waitRead(int timeLimit);
  private:
};

#endif
