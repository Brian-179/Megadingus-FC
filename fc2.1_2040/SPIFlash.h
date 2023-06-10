#include <SPI.h>

class SPIFlash {
  private:
    byte chipSelectPin;
  
  public:
    SPIFlash(byte csPin) {
      chipSelectPin = csPin;
      pinMode(chipSelectPin, OUTPUT);
      digitalWrite(chipSelectPin, HIGH);  // Disable chip select by default
    }

    void write(uint32_t address, byte* data, uint32_t length) {
      digitalWrite(chipSelectPin, LOW);  // Enable chip select
      SPI.transfer(0x06);  // Send Write Enable command
      digitalWrite(chipSelectPin, HIGH);  // Disable chip select

      digitalWrite(chipSelectPin, LOW);  // Enable chip select
      SPI.transfer(0x02);  // Send Page Program command
      SPI.transfer((address >> 16) & 0xFF);  // Send high byte of address
      SPI.transfer((address >> 8) & 0xFF);   // Send middle byte of address
      SPI.transfer(address & 0xFF);  // Send low byte of address

      for (uint32_t i = 0; i < length; i++) {
        SPI.transfer(data[i]);  // Send data byte
      }

      digitalWrite(chipSelectPin, HIGH);  // Disable chip select
    }

    void read(uint32_t address, byte* buffer, uint32_t length) {
      digitalWrite(chipSelectPin, LOW);  // Enable chip select
      SPI.transfer(0x03);  // Send Read command
      SPI.transfer((address >> 16) & 0xFF);  // Send high byte of address
      SPI.transfer((address >> 8) & 0xFF);   // Send middle byte of address
      SPI.transfer(address & 0xFF);  // Send low byte of address

      for (uint32_t i = 0; i < length; i++) {
        buffer[i] = SPI.transfer(0x00);  // Read data byte
      }

      digitalWrite(chipSelectPin, HIGH);  // Disable chip select
    }

    void print(uint32_t address, uint32_t length) {
      byte buffer[length];
      read(address, buffer, length);

      for (uint32_t i = 0; i < length; i++) {
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
      }

      Serial.println();
    }
};
