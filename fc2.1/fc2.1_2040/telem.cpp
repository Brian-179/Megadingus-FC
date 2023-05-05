#include <SPI.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include "Aflightlibs.h"
#include "telem.h"

int espcs;
int serialPin;
int baud = 38400; //default baud is 38400

//SoftwareSerial ESPSerial(); //8

SoftwareSerialOneWire::SoftwareSerialOneWire(int serialpin, int Baud) {
  serialPin = serialpin;
  baud = Baud;
  //SoftwareSerial ESPSerial(7, serialPin);
  //ESPSerial.begin(baud);
}

boolean SoftwareSerialOneWire::available() {
  SoftwareSerial ESPSerial(serialPin, 7); //8
  ESPSerial.begin(baud);
  return (ESPSerial.available());
}

void SoftwareSerialOneWire::println(String input) {
    SoftwareSerial ESPSerial(7, serialPin); //8
    ESPSerial.begin(baud);
    ESPSerial.println(input);
    ESPSerial.end();
  //ESPSerial.println(input);
}

String waitRead() {
  SoftwareSerial ESPSerial(serialPin, 7); //8
  ESPSerial.begin(baud);
  boolean recieved = false;
  while (!recieved) {
    if (ESPSerial.available()) {
      return (String(ESPSerial.read()));
      recieved = true;
    }
  }
  return ("");
}

String waitRead(int timeLimit) {
  SoftwareSerial ESPSerial(serialPin, 7); //8
  ESPSerial.begin(baud);
  boolean recieved = false;
  int olderMillis = millis();
  while (!recieved) {
    if ((millis() - olderMillis) >= timeLimit) {
      break;
    }
    else if (ESPSerial.available()) {
      return (String(ESPSerial.read()));
      recieved = true;
    }
  }
  return ("");
}


String SoftwareSerialOneWire::read() {
  SoftwareSerial ESPSerial(serialPin, 7); //8
  ESPSerial.begin(baud);
  return (String(ESPSerial.read()));
  ESPSerial.end();
}

SPITelem::SPITelem(int Espcs) {
  //  SPI.setRX(4);
  //  SPI.setTX(3);
  //  SPI.setSCK(2);
  //  SPI.begin();
  espcs = Espcs;
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

String SPITelem::receive() {
  String rxdata;
  digitalWrite(espcs, LOW);
  while (SPI.transfer(1)) {
    rxdata += SPI.transfer(1);
  }
  digitalWrite(espcs, HIGH);
  return rxdata;
}

void readData(uint8_t *data) {
  digitalWrite(espcs, LOW);
  SPI.transfer(0x03);
  SPI.transfer(0x00);
  for (uint8_t i = 0; i < 127; i++) {
    data[i] = SPI.transfer(0);
  }
  digitalWrite(espcs, HIGH);
}

String SPITelem::read() {
  char data[128];
  data[127] = 0;
  readData((uint8_t *)data);
  return String(data);
}

void writeData(uint8_t *data, size_t len) {
  uint8_t i = 0;
  digitalWrite(espcs, LOW);
  SPI.transfer(0x02);
  SPI.transfer(0x00);
  while (len-- && i < 127) {
    SPI.transfer(data[i++]);
  }
  while (i++ < 127) {
    SPI.transfer(0);
  }
  digitalWrite(espcs, HIGH);
}

void SPITelem::send(String input) {
  digitalWrite(espcs, LOW);
  SPI.transfer(0x02);
  SPI.transfer(0x00);
  for (int i = 0; i < sizeof(input); i++) {
    SPI.transfer(input[i]);
  }
  //writeData((uint8_t *)input, strlen(input));
  digitalWrite(espcs, HIGH);
}

void SPITelem::send(float input) {
  digitalWrite(espcs, LOW);
  SPI.transfer(0x02);
  SPI.transfer(0x00);
  for (int i = 0; i < sizeof(input); i++) {
    SPI.transfer(input);
  }
  //writeData((uint8_t *)input, strlen(input));
  digitalWrite(espcs, HIGH);
}

void SPITelem::send(Telem input) {
  digitalWrite(espcs, LOW);
  SPI.transfer(0x02);
  SPI.transfer(0x00);
  size_t inputSize = sizeof(input);
  uint8_t inputArray[inputSize];
  memcpy(inputArray, &input, inputSize);
  for (int i = 0; i < sizeof(input); i++) {
    SPI.transfer(inputArray[inputSize]); //needs to be worked on
  }
  //writeData((uint8_t *)input, strlen(input));
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
