//for Brian Wong's FC2.1 TVC Flight controller, compile for RP2040/raspberry pi pico (Earle Philhower).
//"generic RP2040"
//16mb flash, 8mb sketch 8mb FS
//W25Q128JV QSPI /4

//pyros are gpio10-12
//servos are gpio13-15
//tx and rx 0 are on gpio16 and 17 respectively
//rgb led is on gpio18-20
//buzzer is on gpio9
//esp_cs is on gpio8
//sd_cs is on gpio6
//flashcs is on gpio5
//sd_detect is on gpio25
//vin is on gpio26/adc0

//Kp is divided by 1000, for the sake of easy tuning

//4096B = ~34s of altitude datalogging.

//TODO: low pass filter for gyro (maybe accel) typedef struct for telemetry, computing servoangles in loop(), new AHRS system, cleaning up code, redo datalogging system, config via sd card and usb serial, fix reading vin
//The SD library in the rp2040 thing can conflict with the adafruit fork of SdFat, thus causing a failure to compile.

#include "SD.h"
#include <SoftwareSerial.h>
#include "telem.h"

//SPITelem ESP(8);
//SoftwareSerialOneWire ESPSerial(8, 38400);

SoftwareSerial ESPSerial(7, 8);

const boolean useProp = false;
const boolean groundTest = false; //static holddown test

const boolean copyToSD = false; //copy datalog from internal QSPI flash to MicroSD card oncce landed
const boolean copyToFlash = false; //copy datalog from microSD card to internal QSPI flash once landed

const boolean logToFlash = false; //datalogging to internal QSPI flash. Can cause spikes of latency in both cores; LittleFS halts both cores to write to flash. Logging to other SPI flash is work in progress, that will hopefully solve this issue.
const boolean logToSD = true; //datalogging to MicroSD card.

const float imuGain = 0.01; //imu fusion thing

const double defaultP = 4.00;
const double defaultI = 0.1;
const double defaultD = 0.10;

const float defaultDelay = 3.00;

const float referencePressure = 1017.0;// hPa local QFF (official meteor-station reading)
const float outdoorTemp = 25; // °C  measured local outdoor temp.
const float barometerAltitude = 307.0;  // meters ... map readings + barometer position, mullaley NSW is 307m

const float pyroTime = 250; //pyro channel on time (ms)

float centrepointX = 65; //servo centrepoint,usually 90 (adjust for your own setup)
float centrepointY = 95;

float servoLimit = 20; //servo actuation limit on either side e.g. 10deg = 10deg on either side = 20deg total
float propThrust = 50; //prop's thrust for ground testing (should be enough to get control with TVC)
float propBurnTime = 3000;

const int SDCS = 6;
const int flashCS = 5;
const int ESPCS = 8;

const int countdownTime = 10;

//int ESPBaud = 38400; //baudrate between rp2040 and esp8266

//#include <SdFat.h>
//#include "Adafruit_SPIFlash.h"
#include <LittleFS.h>
//#include <SerialFlash.h>
#include "Aflightlibs.h"
#include <MadgwickAHRS.h>
#include <hardware/flash.h>
#include "EEPROM.h"
//#include <SPIMemory.h>
//#include <PID_v1.h>
#include <SPI.h>
//#include <SDFS.h>
#include <EnvironmentCalculations.h>
#include <BME280I2C.h>
#include <Servo.h>
#include "BMI088.h"
//#include "SPIFlash.h"
#include "Arduino.h"
//#include "SdFatConfig.h"
//#include "ESP8266SdFat\SdFat.h"
//#include <Adafruit_SPIFlash.h>
#include <Mouse.h>

//#include "ff.h"
//#include "diskio.h"

#define DISK_LABEL    "EXT FLASH"

//#include "flash_config.h"

//Adafruit_SPIFlash spiflash(&flashTransport);

//Adafruit_SPIFlash flash();

//FatVolume fatfs;

//FATFS elmchamFatfs;

uint8_t workbuf[4096]; // Working buffer for f_fdisk function.

#define FILE_NAME      "datalog.txt"

//#define ADAFRUIT_FLASHTRANSPORT_RP2040_H_
//#define _LFS_LOGLEVEL_          4
//#define FILE_NAME      "datalog.txt"

float datalogInterval = 1000; //datalogging rate

//bool setRX(4);
//bool setCS(8);
//bool setSCK(2);
//bool setTX(3);

double Kp;  // PID loop values
double Ki;
double Kd;

//double Kp = 0.5;
//double Ki = 0;
//double Kd = 0.033;

int startAddr = 12017; //EEPROM writing start address for datalogging.

//double Setpoint = 90;

float targetAnglex = 0;
float targetAnglez = 0;

float delayPeriod; //backup delay period if no baro (from burnout to ejection)

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
float oldMillis2;
float oldMillis3;
float oldMillis4 = 0;
float cycleTime;
float groundAlt = 0;
//float vin;
int runMode = 0; //
int altcount = 0;
int pyro1 = 10;
int pyro2 = 11;
int pyro3 = 12;
int addr;
int readAddr = 0;
float altitude;
unsigned long currMillis;
unsigned long oldMillis;
boolean SDCard = true;
boolean led = false;
boolean datalogging;
boolean lockGyro = false;
float vSpeed;
boolean seenSerialInput;

boolean statWrite;
boolean SDAvailable = true;
float oldMillis6; //datalog stuff
String serialInput;

float vspeed;
float servoanglex;
float servoanglez;
float Alt;
float smoothedalt;
float ax, ay, az;
float gx, gy, gz;
float Vin;

String datalogString;
Telem telem;
String lastEvent;

Servo servox;
Servo servoy;
Servo prop;
//Servo parachute;

//File flash;
//SPIFlash exflash(5);

TVCClass TVC(true, true);

//File file1;

//PID pidx(&anglex, &anglex1, &Setpoint, Kp, Ki, Kd, DIRECT);
//PID pidz(&anglez, &anglez1, &Setpoint, Kp, Ki, Kd, DIRECT);

Bmi088Accel accel(Wire, 0x18);
Bmi088Gyro gyro(Wire, 0x68);

Madgwick filter;

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

//LittleFSConfig cfg;
//cfg.setAutoFormat(false);
//LittleFS.setConfig(cfg);

BME280I2C bme(settings);
//SPIFlash flash;
//Sd2Card card;
//SdVolume volume;
//SdFile piss;

void setup() {
  Serial.begin(115200);
  Serial.print("angle test");

  addr = startAddr;

  EEPROM.begin(98304);  //98304

  //flashSetup();

  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();

  //  SPI.setRX(4);
  //  SPI.setTX(3);
  //  SPI.setSCK(2);

  //SPI.begin();
  //SPI.beginTransaction();

  int status;
  status = accel.begin();
  status = gyro.begin();

  filter.begin(100); // update rate in Hz

  oldMillis2 = 5;
  oldMillis3 = 5;

  //pidx.SetMode(AUTOMATIC);
  //pidz.SetMode(AUTOMATIC);

  pinMode(26, INPUT); //vin / adc0

  ledr(0);
  ledg(0);
  ledb(0);

  //  Kp = (EEPROM.read(0));
  //  Ki = (EEPROM.read(1));
  //  Kd = (EEPROM.read(2));

  //  delayPeriod = (EEPROM.read(3)); //backup delay period if no baro (from burnout to ejection)

  //pinMode(pyro1,OUTPUT); //pyro1
  //file1 = SD.open("piss.txt", FILE_WRITE);

  servox.attach(13);
  servoy.attach(14);
  servox.write(90);
  servoy.write(90);

  if (useProp == true) {
    prop.attach(15);
    prop.write(0);
  }
  //parachute.attach(15);
  //parachute.write(0);

  //  parachute.attach(15);
  //  parachute.write(0);

  if (!gyro.begin()) {
    Serial.println("no gyro");
    digitalWrite(18, LOW);
    lastEvent = "no gyro";
    runMode = -1;
  }

  while (!bme.begin()) {
    Serial.println("no baro");
    digitalWrite(18, LOW);
    lastEvent = "no baro";
    runMode = -1;
  }

  //  if (!SD.begin(SDCS)) {
  //    Serial.println("no sd card");
  //    SDAvailable = false;
  //    tone(9, 200, 200);
  //    delay(200);
  //  }

  //LittleFS.begin();

  //  File file = SD.open("datalog.txt", FILE_WRITE);
  //  //File data = LittleFS.open("/data.txt","w+");
  //
  //  file.println();
  //  file.println();
  //  file.println();
  //  file.println();
  //  file.println();
  //  file.close();

  //  if (!SD.begin(PB1)){
  //    Serial.println("No SD card!");
  //    SDCard = false;
  //  }

  //gravity = accel.getAccelY_mss();

  //EEPROM.end();

  delay(200);

  if (alt() <= -4000) {
    watchdog_enable(1, 1);
    while (1);
  }

  float buzTone = 1000;

  //  for (int i = 0; i <= 1000; i+= 1){
  //    float delayP = 2.5;
  //    buzTone -= 1;
  //    tone(9,buzTone,delayP);    //buzzer startup beep
  //    delay(delayP);
  //  }

  tone(9, 300, 200);
  delay(200);
  tone(9, 400, 200);
  delay(200);
  tone(9, 500, 200);

  ledr(1);
  ledg(1);
  groundAlt = alt();
  delay(200);
  servox.write(centrepointX - 10);
  delay(100);
  servox.write(centrepointX);
  delay(100);
  servox.write(centrepointX + 10);
  delay(100);
  servox.write(centrepointX);
  delay(100);
  servoy.write(centrepointY - 10);
  delay(100);
  servoy.write(centrepointY);
  delay(100);
  servoy.write(centrepointY + 10);
  ledr(0);
  delay(100);
  servoy.write(90);
  ledg(0);
  servox.write(centrepointX);
  servoy.write(centrepointY);
  EEPROM.end();
}

void loop() {
  //rp2040.idleOtherCore();
  accel.readSensor();
  gyro.readSensor();
  TVC.estimateAngles(runMode, imuGain, accel.getAccelX_mss(), accel.getAccelY_mss(), accel.getAccelZ_mss(), gyro.getGyroX_rads(), gyro.getGyroY_rads(), gyro.getGyroZ_rads());

  anglex = TVC.pitch();
  angley = TVC.roll();
  anglez = TVC.yaw();

  currMillis = millis();
  cycleTime = (currMillis - oldMillis2);
  oldMillis2 = millis();

  vSpeed = baroVel();
  alt();
  smoothedAlt();
  ax = accel.getAccelX_mss();
  ay = accel.getAccelY_mss();
  az = accel.getAccelZ_mss();
  gx = gyro.getGyroX_rads();
  gy = gyro.getGyroY_rads();
  gz = gyro.getGyroZ_rads();
  Vin = vin();

  accelSpeedy = (((accelSpeedy + (accel.getAccelY_mss())) - gravity));
  accelAlt = (accelAlt + accelSpeedy);

  vspeed = vSpeed;
  servoanglex = servoAnglex();
  servoanglez = servoAnglez();
  Alt = alt();
  smoothedalt = smoothedAlt();

  //rp2040.idleOtherCore();

  datalogString = (String(currMillis) + ": " + String(runMode) + " " + String(TVC.pitch()) + " " + String(TVC.roll()) + " " + String(TVC.yaw()) + " " + String(servoanglex - centrepointX) + " " + String(servoanglez - centrepointY) + " " + "alt:" + " " + String(Alt) + " " + String(smoothedalt) + " " + String(vSpeed) + " " + String(Vin) + " " + "raw accel readings:" + " " + String(ax) + " " + String(ay) + " " + String(az) + " filtered az: " + String(TVC.filteredAccelY()) + " " + "gyro:" + " " + String(gx * RAD_TO_DEG) + " " + String(gy * RAD_TO_DEG) + " " + String(gz * RAD_TO_DEG) + " " + String(cycleTime) + " " + lastEvent);

  //  telem.currMillis = currMillis;
  //  telem.runMode = runMode;
  //  telem.Anglex = anglex;
  //  telem.Angley = angley;
  //  telem.Anglez = anglez;
  //  telem.servoanglex = servoanglex - 90;
  //  telem.servoanglez = servoanglez - 90;
  //  telem.Alt = Alt;
  //  telem.smoothedalt = smoothedalt;
  //  telem.vSpeed = vSpeed;
  //  telem.Vin = Vin;
  //  telem.ax = ax;
  //  telem.ay = ay;
  //  telem.az = az;
  //  telem.gx = gx;
  //  telem.gy = gy;
  //  telem.gz = gz;
  //  telem.cycleTime = cycleTime;
  //  telem.lastEvent = lastEvent;

  //rp2040.resumeOtherCore();

  //  anglex = (anglex + (gyro.getGyroX_rads() * (M_PI / 180) * ((4078/71) * 1.5)));
  //  angley = (angley + (gyro.getGyroY_rads() * (M_PI / 180) * ((4078/71) * 1.5)));
  //  anglez = (anglez + (gyro.getGyroZ_rads() * (M_PI / 180) * ((4078/71) * 1.5)));

  //  anglex = (((anglex + (gyro.getGyroX_rads())) * 4068) /71);
  //  angley = (((anglex + (gyro.getGyroX_rads())) * 4068) /71);
  //  anglez = (((anglex + (gyro.getGyroX_rads())) * 4068) /71);

  if (datalogging == true) {
    //exFlashDatalog();
    //flashDatalog();
    //    rp2040.fifo.push(1);
    //    rp2040.fifo.push(currMillis);
    //    rp2040.fifo.push(anglex);
    //    rp2040.fifo.push(angley);
    //    rp2040.fifo.push(anglez);
    //    rp2040.fifo.push(servoAnglex() - centrepoint);
    //    rp2040.fifo.push(servoAnglez() - centrepoint);
    //    rp2040.fifo.push(alt());
    //    rp2040.fifo.push(smoothedAlt());
    //    rp2040.fifo.push(vSpeed);
    //    rp2040.fifo.push(vin());
    //    rp2040.fifo.push(ax);
    //    rp2040.fifo.push(ay);
    //    rp2040.fifo.push(az);
    //    rp2040.fifo.push(gx);
    //    rp2040.fifo.push(gy);
    //    rp2040.fifo.push(gz);
  }
  if (Serial.available()) { //serial interface
    //String serialInput;
    serialInput = Serial.readString();
    serialInput.trim();
    if (serialInput == "dump") {
      dump();
    }
    //    if (serialInput == "dumpFlash"){
    //      File data = LittleFS.open("/data.txt","r");
    //      while (data.available()){
    //        Serial.write(data.read());
    //      }
    //    }
    //    if (serialInput == "clearSD") {
    //      SD.rmdir("/");
    //      if (SD.rmdir("/") == 1) {
    //        Serial.println("done");
    //      }
    //      else {
    //        Serial.println("error of some sort, im sorry that its not helpful");
    //      }
    //    }
    //    if (serialInput == "formatSD") {
    //      SDFS.format();
    //      Serial.println("done");
    //    }
    //    if (serialInput == "formatFlash"){
    //      LittleFS.format();
    //      Serial.println("formatted flash!");
    //    }
    if (serialInput == "SDInfo") {
      SDInfo();
    }
    if (serialInput == "dump_telem") {
      readAddr = startAddr;
      dump();
    }
    if ((serialInput == "clr_eeprom") or (serialInput == "eeprom_clr")) {
      addr = 0;
      wipe();
    }
    if ((serialInput == "clr_datalog") or (serialInput == "datalog_clr")) {
      addr = startAddr;
      wipe();
    }
    if (serialInput == "eepromtoSD") {
      //eepromtoSD();
    }

    if (serialInput == "reboot") {
      watchdog_enable(1, 1);
      while (1);
    }
    if (serialInput == "save") {
      save();
    }
    if (serialInput == "set") {
      set(serialInput);
    }
    //    if (serialInput == "defaults") {
    //      EEPROM.write(0, defaultP);
    //      EEPROM.write(1, defaultI);
    //      EEPROM.write(2, defaultD);
    //      EEPROM.write(3, defaultDelay);
    //      Serial.println("reset settings to defaults");
    //    }
    if (serialInput == "getpids") {
      Serial.println(Kp);
      Serial.println(Ki);
      Serial.println(Kd);
      Serial.println(delayPeriod);
    }
    if (serialInput == "cli") {
      runMode = -2;
      ledr(1);
      ledg(1);
      ledb(1);
      Serial.println("CLI mode!");
      Serial.println("reboot to exit");
    }
    if (serialInput == "landed") {
      runMode = 4;
    }
    if (serialInput == "waiting") {
      runMode = 0;
    }
    if (serialInput == "poweredAscent") {
      runMode = 1;
    }
    if (serialInput == "coast") {
      runMode = 2;
    }
    if (serialInput == "descent") {
      runMode = 3;
    }
    if (serialInput == "testAllPyros"){
      Serial.println("Pyros will be held ON until otherwise");
      digitalWrite(pyro1, HIGH);
      digitalWrite(pyro2, HIGH);
      digitalWrite(pyro3, HIGH);
    }
    if (serialInput == "allPyrosOff"){
      Serial.println("All pyro channels OFF");
      digitalWrite(pyro1, LOW);
      digitalWrite(pyro2, LOW);
      digitalWrite(pyro3, LOW);
    }
    if (serialInput == "pyro1") {
      //TVC.pyro1();
      ledr(1);
      ledg(0);
      ledb(0);
      digitalWrite(pyro1, HIGH);
      delay(250);
      digitalWrite(pyro1, LOW);
      ledr(0);
      ledg(1);
      ledb(1);
      Serial.println("fired pyro channel 1 (hopefully)");
    }
    if (serialInput == "pyro2") {
      TVC.pyro2();
    }
    if (serialInput == "pyro3") {
      TVC.pyro3();
    }
    if (serialInput == "servox") {
      servox.write(centrepointX + servoLimit);
      delay(100);
      servox.write(centrepointX - servoLimit);
      delay(100);
      servox.write(centrepointX);
    }
    if (serialInput == "servoy") {
      servoy.write(centrepointY + servoLimit);
      delay(100);
      servoy.write(centrepointY - servoLimit);
      delay(100);
      servoy.write(centrepointY);
    }
    if (serialInput == "dumpFlash") {
      int oldrunmode = runMode;
      runMode = -2;
      File data = LittleFS.open("/data.txt", "r");
      Serial.println("dumping flash:");
      while (data.available()) {
        Serial.write(data.read());
      }
      data.close();
      runMode = oldrunmode;
      //serialInput = "";
    }
    if (serialInput == "defaults") {
      LittleFS.remove("/config.txt");
      File file = LittleFS.open("/config.txt", "a");
      file.println(defaultP);
      file.println(defaultI);
      file.println(defaultD);
      file.println(defaultDelay);
      file.close();
      rp2040.reboot();
    }
    if ((serialInput == "deleteTelem") or (serialInput == "clearTelem")) {
      if (LittleFS.exists("/config.txt")) {
        LittleFS.remove("/config.txt");
        Serial.println("done");
      }
      else {
        Serial.println("the config file for some unhelpful reason does not appear to exist");
      }
      //serialInput = "";
    }
    if ((serialInput == "dumpConfig") or (serialInput == "getConfig")) {
      int oldrunmode = runMode;
      runMode = -2;
      File file = LittleFS.open("config.txt", "r");
      Serial.println("dumping config from flash");
      while (file.available()) {
        Serial.write(file.read());
      }
      file.close();
      runMode = oldrunmode;
      //serialInput = "";
    }
    //  if (BOOTSEL) {
    //    buz(1);
    //    runMode = 4;
    //  }
    //  if (!BOOTSEL) {
    //    buz(0);
    //  }
    if (serialInput == "flashtoSD") {
      flashtoSD();
      //serialInput = "";
    }
    if (serialInput == "SDtoFlash" or serialInput == "SDtoflash") {
      SDtoflash();
    }
    if ((serialInput == "SDDump") or (serialInput == "dumpSD")) {
      SDDump();
      //serialInput = "";
    }
    if (serialInput == "formatExternalFlash") {
      //formatExternalFlash();
      //serialInput = "";
    }
    //  if (serialInput == "flashtoSD") {
    //    int oldrunmode = runMode;
    //    runMode = -2;
    //    File data = LittleFS.open("/data.txt", "r");
    //    File file = SD.open("flash.txt", FILE_WRITE);
    //    while (data.available()) {
    //      file.write(data.read());
    //      //rp2040.fifo.push(data.read());
    //      //fifo = data.read();
    //    }
    //  data.close();
    //  file.close();
    //  Serial.println("Copied all data from internal Flash to MicroSD card");
    //  runMode = oldrunmode;
    //}
    if (serialInput == "formatFlash") {
      LittleFS.format();
      Serial.println("formatted flash!");
      //serialInput = "";
    }
    if (serialInput == "clearFlashDatalog") {
      LittleFS.remove("/data.txt");
      Serial.println("done");
    }
    if (serialInput == "wipeFlash") {
      LittleFS.rmdir("/");
      Serial.println("done");
    }
    //    if (serialInput == "clearExternalFlash") {
    //      fatfs.rmdir("/");
    //      Serial.println("done");
    //      //serialInput = "";
    //    }
    //  if (serialInput == "formatExternalFlash") {
    //    // Make filesystem.
    //    FRESULT r = f_mkfs("", FM_FAT, 0, workbuf, sizeof(workbuf));
    //    if (r != FR_OK) {
    //      Serial.print("Error, f_mkfs failed with error code: "); Serial.println(r, DEC);
    //      while (1) yield();
    //    }
    //    // mount to set disk label
    //    r = f_mount(&elmchamFatfs, "0:", 1);
    //    if (r != FR_OK) {
    //      Serial.print("Error, f_mount failed with error code: "); Serial.println(r, DEC);
    //      while (1) yield();
    //    }
    //    // Setting label
    //    Serial.println("Setting disk label to: " DISK_LABEL);
    //    r = f_setlabel(DISK_LABEL);
    //    if (r != FR_OK) {
    //      Serial.print("Error, f_setlabel failed with error code: "); Serial.println(r, DEC);
    //      while (1) yield();
    //    }
    //    // unmount
    //    f_unmount("0:");
    //    // sync to make sure all data is written to flash
    //    flash.syncBlocks();
    //    Serial.println("Formatted flash!");
    //    if (!fatfs.begin(&flash)) {
    //      Serial.println("Error, failed to mount newly formatted filesystem!");
    //      while (1) delay(1);
    //    }
    //  }
    //    if (serialInput == "dumpExternalFlash") {
    //      Serial.println("dumping external flash");
    //      File32 flash = fatfs.open("datalog.txt", FILE_READ);
    //      while (flash.available()) {
    //        Serial.write(flash.read());
    //      }
    //      //serialInput = "";
    //    }
    if (serialInput == "gimbalTest") {
      Serial.println("going into gimbalTest mode, adding accel into angles");
      runMode = 69;
    }
    if (serialInput == "countdown") {
      Serial.println("I will count down from 10, then go into poweredAscent mode");
      runMode = -3;
    }
    if (serialInput == "calibrateESC") {
      Serial.println("make sure the esc on the servo3 connector is unplugged from VBAT");
      prop.write(0);
      delay(2000);
      Serial.println("plug in the ESC and wait");
      prop.write(180);
      delay(15000);
      prop.write(0);
    }
    if (serialInput == "testProp") {
      Serial.println("testing prop");
      prop.write(10);
      delay(5000);
      prop.write(0);
    }
    if (serialInput == "testPropInf") {
      Serial.println("the prop will run until stopProp is run");
      prop.write(10);
    }
    if (serialInput == "stopProp") {
      Serial.println("stopping prop");
      prop.write(0);
    }
    if (serialInput == "propSpeedTest") {
      Serial.println("prop speed will be ramped up from 0 to max and back");
      int propSpeed = 0;
      while (propSpeed <= 50) {
        propSpeed += 1;
        prop.write(propSpeed);
        delay(30);
      }
      while (propSpeed >= 0) {
        propSpeed -= 1;
        //        if (propSpeed <= 0){
        //          propSpeed = 0;
        //        }
        prop.write(propSpeed);
        delay(30);
      }
      prop.write(0);
    }
    //serialInput = "";
    if (serialInput == "formatSD") {
      SDFS.format();
      Serial.println("done");
    }
    if (serialInput == "wipeSD") {
      SD.remove("datalog.txt");
      SD.rmdir("/");
      Serial.println("done");
    }
    if ((serialInput == "autoClicker") or (serialInput == "autoclicker")) {
      Mouse.begin();
      while (1) {
        Mouse.click();
      }
    }
    //  if (serialInput == "eepromtoSD"){
    //    eepromtoSD();
    //  }
    //serialInput = ""; //flush the buffer
  }

  //rp2040.idleOtherCore();
  //  if (BOOTSEL) {
  //    buz(1);
  //    runMode = 4;
  //  }
  //  if (!BOOTSEL) {
  //    buz(0);
  //  }
  //rp2040.resumeOtherCore();

  //runmodes
  if (runMode == -3) {
    countdown();
  }
  if (runMode == -2) { //cli mode
    //cli();
    //cmd();
  }
  if (runMode == -1) { //error 1

  }
  if (runMode == 0) {
    wait();
  }
  if (runMode == 1) {
    poweredAscent();
  }
  if (runMode == 2) {
    coast();
  }
  if (runMode == 3) {
    descent();
  }
  if (runMode == 4) {
    //runMode = 0;
    //rp2040.reboot(); //reboot in case of accidental launch detect or multiple flights
    landed();
  }
  if (runMode == 69) {
    gimbalTest();
  }

  //  Serial.println(alt());
  if (runMode >= 0) {
    //SPI.transfer(alt());
    //Serial.println(EEPROM.length());
    //    Serial.print(anglex);
    //    Serial.print("     ");
    //    Serial.print(angley);
    //    Serial.print("     ");
    //    Serial.print(anglez);
    //    Serial.print("     ");
    //    //Serial.print(cycleTime);
    //    //Serial.print("     ");
    //    //Serial.print(accelSpeedy);
    //    //Serial.print("     ");
    //    //Serial.print(accelAlt);
    //    Serial.print(servoAnglex() - centrepoint);
    //    Serial.print("     ");
    //    Serial.print(servoAnglez() - centrepoint);
    //    Serial.print("     ");
    //    Serial.print("alt: ");
    //    Serial.print("     ");
    //    Serial.print(alt());
    //    Serial.print("     ");
    //    Serial.print(smoothedAlt());
    //    Serial.print("     ");
    //    Serial.print(vSpeed);
    //    Serial.print("     ");
    //    Serial.print(vin());
    //    Serial.print("     ");
    //    Serial.print(cycleTime);
    //    Serial.print("     ");
    //    Serial.print(datalogString);
    //    Serial.println();
  }

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
  //oldMillis2 = currMillis;
  //oldMillis3 = currMillis;
  //delay(100);
}

void setup1() {
  //ESPSerial.begin(38400);
  //  SPI.setRX(4);
  //  SPI.setTX(3);
  //  SPI.setSCK(2);
  //
  //  SPI.begin();
  //
  SPISetup();
  //exFlashSetup();
  flashSetup();
  SDSetup();

  //telemSetup(8);

  if (LittleFS.exists("/config.txt")) {
    File file = LittleFS.open("/config.txt", "r");
    Kp = file.parseFloat();
    Ki = file.parseFloat();
    Kd = file.parseFloat();
    delayPeriod = file.parseFloat();
    file.close();
  }
  else {
    File file = LittleFS.open("/config.txt", "a");
    file.println(defaultP);
    file.println(defaultI);
    file.println(defaultD);
    file.println(defaultDelay);
    file.close();
    Kp = defaultP;
    Ki = defaultI;
    Kd = defaultD;
  }

  //  while (BOOTSEL){
  //    runMode = 0; //set runmode to "waiting" if bootsel pressed (can be used for arming)
  //  }

  //  EEPROM.begin(98304);
  //  Kp = (EEPROM.read(0));
  //  Ki = (EEPROM.read(1));
  //  Kd = (EEPROM.read(2));
  //
  //  delayPeriod = (EEPROM.read(3)); //backup delay period if no baro (from burnout to ejection)
  //
  //
  //  if (!SD.begin(SDCS)){
  //    Serial.println("no sd card");
  //    SDAvailable = false;
  //    tone(9,200,200);
  //    delay(200);
  //  }
  //
  //  File file = SD.open("datalog.txt", FILE_WRITE);
  //  File data = LittleFS.open("/data.txt","w+");
  //
  //  file.println();
  //  file.println();
  //  file.println();
  //  file.println();
  //  file.println();
  //  file.close();
}

byte fifo;

void loop1() {

  //servoanglex();
  //servoangley();

  if (runMode == -2) {
    cmd();
  }
  //ESPSerial.println(datalogString);
  if (runMode >= 0) {
    Serial.println(datalogString); //has to be on the second core since if it's on the first core that somehow breaks estimateAngles();
  }
  //ESP.send(datalogString);
  //int ESPMessageSize = sizeof(datalogString);
  //char ESPMessage[ESPMessageSize] = datalogString;
  //ESPSerial.println(datalogString);
  //  int num1 = serialInput.parseInt();
  //  int num2 = serialInput.parseInt();
  //  if ((num1 != 0) & (num2 != 0)){
  //
  //  }
  //telemSend();
  //  if (serialInput == "formatExflash"){
  //    SerialFlash.eraseAll();
  //  }
  //  if (runMode == 3) {
  //    static boolean savedTelem = false;
  //    if (savedTelem == false) {
  //      File32 exflash = fatfs.open("datalog.txt", FILE_READ);
  //      File flash = LittleFS.open("/data.txt", "a");
  //      while (exflash.available()) {
  //        flash.write(exflash.read());
  //      }
  //      flash.close();
  //      exflash.close();
  //      fatfs.rmdir("/");
  //      savedTelem == true;
  //    }
  //  }
  //  if (runMode == 4) {
  //    static boolean savedTelem = false;
  //    if (savedTelem == false) {
  //      File32 exflash = fatfs.open("datalog.txt", FILE_READ);
  //      File flash = LittleFS.open("/data.txt", "a");
  //      while (exflash.available()) {
  //        flash.write(exflash.read());
  //      }
  //      flash.close();
  //      exflash.close();
  //      fatfs.rmdir("/");
  //      savedTelem == true;
  //    }
  //  }

  //  if (ESPSerial.available()) {
  //    if (!isDigit(ESPSerial.peek())) {
  //      serialInput = ESPSerial.readString();
  //      serialInput.trim();
  //    }
  //  }


  if (runMode >= 0) {
    if (datalogging == true) {
      datalog();
      //exFlashDatalog();
      if (logToFlash == true) {
        flashDatalog();
      }
      if (logToSD == true) {
        SDDatalog();
      }
    }
    if (datalogging == false) {
      //data.close();
    }
  }
  //  if (runMode == 3) {
  //    parachute.write(180);
  //  }
  //  if (runMode != 3) {
  //    parachute.write(0);
  //  }
}

//void ESPSend() {
//  ESP.send(currMillis);
//  ESP.send(runMode);
//  ESP.send(anglex);
//  ESP.send(angley);
//  ESP.send(anglez);
//  ESP.send(servoanglex - 90);
//  ESP.send(servoanglez - 90);
//  ESP.send(Alt);
//  ESP.send(smoothedalt);
//  ESP.send(vSpeed);
//  ESP.send(Vin);
//  ESP.send(ax);
//  ESP.send(ay);
//  ESP.send(az);
//  ESP.send(gx * RAD_TO_DEG);
//  ESP.send(gy * RAD_TO_DEG);
//  ESP.send(gz * RAD_TO_DEG);
//  ESP.send(cycleTime);
//  ESP.send(lastEvent);
//}

void cmd() {
  ledr(1);
  ledg(1);
  ledb(1);
  //Serial.println(ESP.read());
}

void SPISetup() {
  SPI.setRX(4);
  SPI.setTX(3);
  SPI.setSCK(2);
  //SPI.beginTransaction(SPISettings(10000000, LSBFIRST, SPI_MODE0));
  SPI.begin();
}

//void exFlashSetup() {
//  //flash.begin();
//  if (!spiflash.begin()) {
//    Serial.println("flash error");
//  }
//  if (!fatfs.begin(&spiflash)) {
//    Serial.println("SPI flash partition error for some reason");
//    formatExternalFlash();
//    rp2040.reboot();
//  }
//  else {
//    Serial.println("spi flash init done");
//  }
//  File32 exflash = fatfs.open("datalog.txt", FILE_WRITE);
//  exflash.println();
//  exflash.println();
//  exflash.println();
//  exflash.println();
//  exflash.println();
//  exflash.close();
//}

void SDSetup() {
  if (!SD.begin(SDCS)) {
    Serial.println("no sd card");
    SDAvailable = false;
    tone(9, 200, 200);
    delay(200);
  }
  File file = SD.open("datalog.txt", FILE_WRITE);
  file.println();
  file.println();
  file.println();
  file.println();
  file.println();
  file.close();
  if (SD.exists("config.txt")) {
    LittleFS.remove("config.txt");
    File file = SD.open("config.txt", FILE_READ);
    File flash = LittleFS.open("config.txt", "a");
    while (file.available()) {
      flash.write(file.read());
    }
  }
}

void flashSetup() {
  LittleFS.begin();
  File flash = LittleFS.open("/data.txt", "a");
  if (flash) {
    flash.println();
    flash.println();
    flash.println();
    flash.println();
    flash.println();
    flash.close();
  }
  else {
    Serial.println("littlefs error");
  }
}

void flashtoSD() {
  File file = SD.open("datalog.txt", FILE_WRITE);
  File flash = LittleFS.open("data.txt", "r");
  while (flash.available()) {
    file.write(flash.read());
  }
  //  while (fifo != -1) {
  //file.write(rp2040.fifo.pop());
  //file.write(fifo);
  //Serial.println("pop");
  //  }
  file.close();
  flash.close();
  Serial.println("Copied all data from internal QPSI flash to the MicroSD card");
  Serial.println("done");
}

void SDtoflash() {
  File file = SD.open("datalog.txt", FILE_READ);
  File flash = LittleFS.open("data.txt", "a");
  while (file.available()) {
    flash.write(file.read());
  }
  file.close();
  flash.close();
  Serial.println("Copied all data from MicroSD card to internal QSPI flash");
  Serial.println("done");
}

//void flashDatalog() {
//  File data = LittleFS.open("/data.txt", "a");
//  if (data) {
//    while (rp2040.fifo.pop() == 1) {
//      data.print(rp2040.fifo.pop());
//      data.print(":     ");
//      data.print(rp2040.fifo.pop());
//      data.print("     ");
//      data.print(rp2040.fifo.pop());
//      data.print("     ");
//      data.print(rp2040.fifo.pop());
//      data.print("     ");
//      data.print(rp2040.fifo.pop());
//      data.print("     ");
//      data.print(rp2040.fifo.pop());
//      data.print("     ");
//      data.print(rp2040.fifo.pop());
//      data.print("     ");
//      data.print("alt: ");
//      data.print("     ");
//      data.print(rp2040.fifo.pop());
//      data.print("     ");
//      data.print(rp2040.fifo.pop());
//      data.print("     ");
//      data.print(rp2040.fifo.pop());
//      data.print("     ");
//      data.print(rp2040.fifo.pop());
//      data.print("     ");
//      data.print("accel raw readings");
//      data.print("     ");
//      data.print(rp2040.fifo.pop());
//      data.print("     ");
//      data.print(rp2040.fifo.pop());
//      data.print("     ");
//      data.print(rp2040.fifo.pop());
//      data.print("     ");
//      data.print("gyro:");
//      data.print("     ");
//      data.print(rp2040.fifo.pop() * RAD_TO_DEG);
//      data.print("     ");
//      data.print(rp2040.fifo.pop() * RAD_TO_DEG);
//      data.print("     ");
//      data.print(rp2040.fifo.pop() * RAD_TO_DEG);
//      data.println();
//      data.close();
//      delay(100);
//    }
//  }
//}

//void flashDatalog() {
//  File poo = LittleFS.open("/data.txt", "a");
//  if (poo) {
//    poo.println(1);
//    poo.close();
//  }
//}

void SDDatalog() {
  if (SDAvailable == true) {
    File file = SD.open("datalog.txt", FILE_WRITE);
    if (file) {
      file.println(datalogString);
      file.close();
    }
  }
}

//void SDDatalog() {
//  static boolean writeMode = false;
//  static float oldMillis69 = 1000;
//  if (millis() - oldMillis69 >= 0) {
//    if (writeMode == true) {
//      if (SDAvailable == true) {
//        File file = SD.open("datalog.txt", FILE_WRITE);
//        if (file) {
//          file.println(datalogString);
////          file.print(currMillis);
////          file.print(":     ");
////          file.print(runMode);
////          file.print("     ");
////          file.print(anglex);
////          file.print("     ");
////          file.print(angley);
////          file.print("     ");
////          file.print(anglez);
////          file.print("     ");
////          file.print(servoanglex - centrepoint);
////          file.print("     ");
////          file.print(servoanglez - centrepoint);
////          file.print("     ");
////          file.print("alt: ");
////          file.print("     ");
////          file.print(Alt);
////          file.print("     ");
////          file.print(smoothedalt);
////          file.print("     ");
////          file.print(vSpeed);
////          file.print("     ");
////          file.print(Vin);
////          file.print("     ");
////          file.print("accel raw readings");
////          file.print("     ");
////          file.print(ax);
////          file.print("     ");
////          file.print(ay);
////          file.print("     ");
////          file.print(az);
////          file.print("     ");
////          file.print("gyro:");
////          file.print("     ");
////          file.print(gx * RAD_TO_DEG);
////          file.print("     ");
////          file.print(gy * RAD_TO_DEG);
////          file.print("     ");
////          file.print(gz * RAD_TO_DEG);
////          file.println();
////          //delay(100);
////          file.close();
//        }
//      }
//      writeMode = false;
//    }
//    if (writeMode == false) {
//      writeMode = true;
//    }
//    oldMillis69 = millis();
//  }
//}

//void exFlashDatalog() {
//  String datalogString2 = datalogString + "     ";
//  static boolean datalogged = false;
//  static int olderMillis = 0;
//  if ((millis() - olderMillis) >= (500 / 2)) {
//    if (datalogged == false) {
//      datalogged = true;
//    }
//    if (datalogged == true) {
//      File32 exflash = fatfs.open("datalog.txt", FILE_WRITE);
//      if (exflash) {
//        exflash.print(datalogString2);
//        //    exflash.print(currMillis);
//        //    exflash.print(":     ");
//        //    exflash.print(runMode);
//        //    exflash.print("     ");
//        //    exflash.print(anglex);
//        //    exflash.print("     ");
//        //    exflash.print(angley);
//        //    exflash.print("     ");
//        //    exflash.print(anglez);
//        //    exflash.print("     ");
//        //    exflash.print(servoanglex - centrepoint);
//        //    exflash.print("     ");
//        //    exflash.print(servoanglez - centrepoint);
//        //    exflash.print("     ");
//        //    exflash.print("alt: ");
//        //    exflash.print("     ");
//        //    exflash.print(Alt);
//        //    exflash.print("     ");
//        //    exflash.print(smoothedalt);
//        //    exflash.print("     ");
//        //    exflash.print(vSpeed);
//        //    exflash.print("     ");
//        //    exflash.print(Vin);
//        //    exflash.print("     ");
//        //    exflash.print("accel raw readings");
//        //    exflash.print("     ");
//        //    exflash.print(ax);
//        //    exflash.print("     ");
//        //    exflash.print(ay);
//        //    exflash.print("     ");
//        //    exflash.print(az);
//        //    exflash.print("     ");
//        //    exflash.print("gyro:");
//        //    exflash.print("     ");
//        //    exflash.print(gx * RAD_TO_DEG);
//        //    exflash.print("     ");
//        //    exflash.print(gy * RAD_TO_DEG);
//        //    exflash.print("     ");
//        //    exflash.print(gz * RAD_TO_DEG);
//        exflash.println();
//        //delay(100);
//        exflash.close();
//      }
//      else {
//        Serial.println("external flash datalogging error");
//      }
//      datalogged = false;
//    }
//    olderMillis = millis();
//  }
//}

void flashDatalog() {
  static boolean savedTelem = false;
  int interval = 500;
  static int olderMillis = 0;
  if (millis() - olderMillis >= interval / 2) {
    if (savedTelem == false) {
      savedTelem = true;
    }
    if (savedTelem == true) {
      File flash = LittleFS.open("/data.txt", "a");
      if (flash) {
        flash.print(datalogString);
        flash.println();
        //delay(100);
        flash.close();
      }
      savedTelem = false;
    }
    olderMillis = millis();
  }
}

void countdown() {
  lastEvent = "counting down for launch";
  static int olderMillis = millis();
  //static int oldestMillis = millis();
  static int countdown = countdownTime;
  static boolean led = true;
  if (countdown >= 1) {
    if (millis() - olderMillis >= 500) {
      if (led == true) {
        ledr(1);
        ledg(0);
        ledb(0);
        tone(9, 500, 500);
        Serial.println(countdown);
        countdown -= 1;
        led = false;
      }
      else {
        ledr(0);
        ledg(0);
        ledb(0);
        led = true;
      }
      olderMillis = millis();
    }
  }
  if (countdown <= 0) { //if more than 10 seconds has elapsed //if (millis() - oldestMillis >= countdown * 1000)
    static boolean Set = false;
    if (Set == false) {
      digitalWrite(pyro2, HIGH); //pyro2 on
      olderMillis = millis();
      Set = true;
    }
    else {
      if (millis() - olderMillis >= pyroTime) {
        tone(9, 1000, 200);
        digitalWrite(pyro2, LOW);
        ledr(0);
        ledg(0);
        ledb(0);
        lockGyro = false;
        addr = startAddr;
        datalogging = true;
        lastEvent = "ignition";
        countdown = countdownTime;
        if (groundTest == true) {
          if (useProp == true) {
            prop.write(propThrust);
          }
        }
        runMode = 1; //poweredAscent mode
      }
    }
  }
}

void wait() {
  lastEvent = "waiting for launch";
  servox.write(centrepointX);
  servoy.write(centrepointY);
  if ((currMillis - oldMillis4) >= 500) {
    oldMillis4 = currMillis;
    if (led == true) {
      //parachute.write(0);
      ledg(1);
      ledr(1);
      ledb(0);
      led = false;
    }
    else {
      ledr(0);
      ledg(0);
      ledb(0);
      led = true;
    }
  }
  boolean foundGravity = false;
  if (foundGravity == false) {
    offset1 = (accel.getAccelY_mss() - gravity);
    foundGravity = true;
  }
  //Serial.println("waiting on pad");
  lockGyro = true; // prevent gyro drift while waiting on pad
  //servoAnglex = 90;
  //servoAnglez = 90;
  accelSpeedy = 0;

  //  if (TVC.filteredAccelY() >= 20) { //accel.getAccelY_mss()
  //    Serial.println("powered ascent");
  //    lockGyro = false;
  //    ledg(1);
  //    ledr(1);
  //    ledb(0);
  //    addr = startAddr;
  //    datalogging = true;
  //    lastEvent = "launch detected!";
  //    runMode = 1;
  //  }
  if (TVC.filteredAccelY() >= 11) {
    static float olderMillis = millis();
    if (olderMillis == 0) {
      olderMillis = millis();
    }
    //if (millis() - olderMillis >= 100) {
    if (TVC.filteredAccelY() >= 11) {
      Serial.println("powered ascent");
      lockGyro = false;
      ledg(1);
      ledr(1);
      ledb(0);
      addr = startAddr;
      datalogging = true;
      lastEvent = "launch detected";
      runMode++;
    }
    //else {
    olderMillis = 0;
    //}
    //}
  }
}

void poweredAscent() {
  datalogInterval = 0;
  servox.write(servoAnglex());
  servoy.write(servoAnglez());

  if (groundTest == true) {
    if (useProp == true) {
      static float propOlderMillis = millis();
      if (propOlderMillis == 0) {
        propOlderMillis = millis();
      }
      if (millis() - propOlderMillis >= propBurnTime) {
        prop.write(0);
        lastEvent = "static test finished";
        propOlderMillis = 0;
        runMode = 4;
      }
    }
  }

  //  if (TVC.filteredAccelY() <= 2 ) {
  //    servox.write(90);
  //    servoy.write(90); //homes and locks servos
  //    Serial.println("coasting");
  //    lastEvent = "coasting";
  //    oldMillis = currMillis;
  //    ledr(1); //purple
  //    ledb(1);
  //    ledg(0);
  //    runMode = 2;
  //  }
  if (accel.getAccelY_mss() <= 5) {
    static float olderMillis = millis();
    if (olderMillis == 0) {
      olderMillis = millis();
    }
    if (millis() - olderMillis >= 100) {
      if (accel.getAccelY_mss() <= 5) {
        Serial.println("coasting");
        servox.write(centrepointX);
        servoy.write(centrepointY);
        oldMillis = currMillis;
        ledg(0);
        ledr(1);
        ledb(1);
        lastEvent = "burnout";
        runMode++;
      }
      else {
        olderMillis = 0;
      }
    }
  }
}

void coast() {
  prop.write(0);
  float altitude2 = alt();
  altcount += 1;
  //Serial.println(altcount);
  //delay(500);
  //if (altcount >=delayPeriod / 2)
  if ((vSpeed <= 0.0) or ((currMillis - oldMillis) >= (delayPeriod * 1000))) {  //(alt() <= (altitude2 - 0.5)
    Serial.println("descending");
    lastEvent = "apogee detected!";
    TVC.pyro1();
    oldMillis = currMillis;
    ledr(0);
    ledg(1);
    ledb(0);
    runMode++;
  }
}

void descent() {
  prop.write(0);
  //TVC.pyro1();
  static float olderMillis = millis();
  static boolean pyro = false;
  static boolean pyroFired = false;
  //File data = LittleFS.open("/data.txt", "a");
  //data.close();
  datalogInterval = 1000;
  //  if (pyroFired == false) {
  //    if (pyro != true) {
  //      parachute.write(180);
  //      digitalWrite(pyro1, HIGH); //pyro charge 1
  //      pyro = true;
  //      olderMillis = millis();
  //    }
  //    if (pyro == true) {
  //      if ((millis() - olderMillis) >= pyroTime) {
  //        parachute.write(0);
  //        digitalWrite(pyro1, LOW); //pyro charge 1 off
  //        pyroFired = true;
  //      }
  //    }
  //  }
  if ((gyro.getGyroX_rads() <= gyroThreshold) & (gyro.getGyroY_rads() <= gyroThreshold) & (gyro.getGyroZ_rads() <= gyroThreshold) & (gyro.getGyroX_rads() >= -gyroThreshold) & (gyro.getGyroY_rads() >= -gyroThreshold) & (gyro.getGyroZ_rads() >= -gyroThreshold)) {
    boolean setOldMillis5;
    int oldMillis5;
    //if (setOldMillis5 != true){
    //oldMillis5 = currMillis;
    //setOldMillis5 = true;
    //}
    //if ((currMillis - oldMillis5) >= 2000){
    Serial.println("landed!");
    lastEvent = "landing detected!";
    datalogging = false;
    //SD.close();
    //EEPROM.commit();
    runMode++;
    //}
  }
}

void landed() {
  prop.write(0);
  ledr(0);
  ledb(0);
  datalogging = false;

  if (SDAvailable == true) {
    if (copyToFlash == true) {
      File file = SD.open("datalog.txt", FILE_READ);
      File flash = LittleFS.open("data.txt", "r");
      flash.println();
      flash.println();
      flash.println();
      flash.println();
      flash.println();
      flash.println("content from MicroSD card:");
      flash.println();
      flash.println();
      flash.println();
      flash.println();
      flash.println();
      while (file.available()) {
        flash.write(file.read());
      }
      file.close();
      flash.close();
    }
    if (copyToSD == true) {
      File file = SD.open("datalog.txt", FILE_READ);
      File flash = LittleFS.open("data.txt", "r");
      file.println();
      file.println();
      file.println();
      file.println();
      file.println();
      file.println("content from internal QSPI flash:");
      file.println();
      file.println();
      file.println();
      file.println();
      file.println();
      while (flash.available()) {
        file.write(flash.read());
      }
      file.close();
      flash.close();
    }
  }

  //  File data = LittleFS.open("/data.txt","r");
  //  File file = SD.open("datalog.txt", FILE_WRITE);
  //  if (SDAvailable){
  //    if (file){
  //      while (data.available()){
  //        file.write(data.read());
  //      }
  //      ledr(1);
  //    }
  //  }
  if ((currMillis - oldMillis4) >= 500) {
    oldMillis4 = currMillis;
    if (led == true) {
      ledg(1);
      led = false;
    }
    else {
      ledg(0);
      led = true;
    }
  }
  //  file.close();
  //  data.close();
}

void gimbalTest() {
  servox.write(servoAnglex());
  servoy.write(servoAnglez());
}

//custom libraries

void SDInfo() {
  //  if (!volume.init(card)){
  //    Serial.println("sd volume error");
  //  }
  //  uint32_t volumeSize;
  //  volumeSize = volume.blocksPerCluster();
  //  volumeSize *= volume.clusterCount();
  //  volumeSize /= 2;
  //  Serial.print("volume size in K:");
  //  Serial.println(volumeSize);
  //  Serial.println("=");
  //  Serial.print(volumeSize / 1024);
  //  Serial.println("MB");
  //  Serial.println("=");
  //  Serial.print((volumeSize / 1024) / 1024);
  //  Serial.println("GB");
  //  root.openRoot(volume);
  //  root.ls(LS_R | LS_DATE | LS_SIZE);
  //  root.close();
}

//void eepromtoSD() {
//  int oldRunMode = runMode;
//  runMode = -2; //temporarily put it into CLI mode, so the cores aren't constantly fighting over the serial terminal
//  static int readaddr = 0;
//  while (readaddr <= EEPROM.length()) {
//    delay(5);
//    File file = SD.open("eeprom.txt", FILE_WRITE);
//    file.print(readaddr);
//    file.print(": ");
//    file.println(EEPROM.read(readaddr));
//    Serial.println(readaddr);
//    readaddr += 1;
//    file.close();
//  }
//  runMode = oldRunMode;
//}

void dump() {
  int oldRunMode = runMode;
  while (readAddr <= EEPROM.length()) {
    Serial.print(readAddr);
    Serial.print("     ");
    Serial.println(EEPROM.read(readAddr));
    readAddr += 1;
    delay(1); //delay to help with printing
  }
  if (readAddr >= EEPROM.length()) {
    Serial.print("that's all ");
    Serial.print(EEPROM.length());
    Serial.println(" bytes of the eeprom dumped!");
    readAddr = 0;
    ledr(1);
    ledg(1);
    ledb(0);
    delay(2000);
    runMode = oldRunMode;
  }
}

void SDDump() {
  int oldRunMode = runMode;
  File file = SD.open("datalog.txt", FILE_READ);
  if (file) {
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close();
  }
  else {
    Serial.println("sd file error");
  }
}

float alt() {
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
  return altitude - groundAlt;
}

//float alt() {
//  float pressure = bme.readPressure() / 100.0F;  // Read pressure in hPa
//  float altitude = 44330.0 * (1.0 - pow(pressure / 1013.25, 0.1903));  // Calculate altitude from pressure
//  return altitude;
//}

float servoAnglex() {
  const float imax = 1000; //maximum cumulative error for preventing I windup.
  static float olderMillis;
  float dt = millis() - olderMillis;
  static float prev_error;
  float error = targetAnglex - anglex;
  static float cumulative_error = 0.0;
  cumulative_error += error * dt;

  if (cumulative_error >= imax) {
    cumulative_error = imax;
  }
  if (cumulative_error <= -imax) {
    cumulative_error = -imax;
  }

  float pval = error * Kp;
  float ival = (cumulative_error * Ki) / 100;

  if (runMode <= 0) {
    cumulative_error = 0;
  }

  float dval = (((error - prev_error) / dt) * Kd) / 10;
  float pidval = centrepointX - (pval + ival + dval);

  if (pidval >= servoLimit + centrepointX) {
    pidval = servoLimit + centrepointX;
  }
  if (pidval <= -servoLimit + centrepointX) {
    pidval = -servoLimit + centrepointX;
  }

  //  Serial.println(pval);
  //  Serial.println(ival);
  //  Serial.println(dval);

  prev_error = error;
  olderMillis = millis();

  return pidval;
}

float servoAnglez() {
  const float imax = 1000; //maximum cumulative error for preventing I windup.
  static float olderMillis;
  float dt = millis() - olderMillis;
  static float prev_error;
  float error = targetAnglez + anglez;
  static float cumulative_error = 0.0;
  cumulative_error += error * dt;

  if (cumulative_error >= imax) {
    cumulative_error = imax;
  }
  if (cumulative_error <= -imax) {
    cumulative_error = -imax;
  }

  float pval = error * Kp;
  float ival = (cumulative_error * Ki) / 100;

  if (runMode <= 0) {
    cumulative_error = 0;
  }

  float dval = (((error - prev_error) / dt) * Kd) / 10;
  float pidval = centrepointY - (pval + ival + dval);

  if (pidval >= servoLimit + centrepointY) {
    pidval = servoLimit + centrepointY;
  }
  if (pidval <= -servoLimit + centrepointY) {
    pidval = -servoLimit + centrepointY;
  }

  prev_error = error;
  olderMillis = millis();

  return pidval;
}

//float servoAnglex(){
//  float error;
//  float cumulative_error;
//  float prev_error;
//  float pval;
//  float ival;
//  float dval;
//  float pidval;
//  error = targetAnglex - anglex;
//  cumulative_error += error;
//  pval = error * Kp;
//  ival = cumulative_error * Ki * (currMillis - oldMillis2);
//  dval = ((error - prev_error) / (currMillis - oldMillis2)) * Kd;
//  pidval = (pval + ival + dval + centrepoint);
//  if (pidval >= servoLimit + centrepoint){
//    pidval = servoLimit + centrepoint;
//  }
//  if (pidval <= -servoLimit + centrepoint){
//    pidval = -servoLimit + centrepoint;
//  }
//  prev_error = error;
//  //Serial.print("     ");
//  //Serial.print(currMillis - oldMillis2);
//  //oldMillis2 = currMillis;
//  return (pidval);
//}
//
//float servoAnglez(){
//  float error;
//  float cumulative_error;
//  float prev_error;
//  float pval;
//  float ival;
//  float dval;
//  float pidval;
//  error = targetAnglez - anglez;
//  cumulative_error += error;
//  pval = error * Kp;
//  ival = cumulative_error * Ki * (currMillis - oldMillis3);
//  dval = (error - prev_error) / (currMillis - oldMillis3) * Kd;
//  pidval = (pval + ival + dval + centrepoint);
//  if (pidval >= servoLimit + centrepoint){
//    pidval = servoLimit + centrepoint;
//  }
//  if (pidval <= -servoLimit + centrepoint){
//    pidval = -servoLimit + centrepoint;
//  }
//  prev_error = error;
//  //Serial.print("     ");
//  //Serial.println(currMillis - oldMillis3);
//  //oldMillis3 = currMillis;
//  return (pidval);
//}




void datalog() {
  //  File file = SD.open("datalog.txt", FILE_WRITE);
  //  static float oldMillis7 = 0;
  //  static boolean statWrite2 = false;
  //  if (file) {
  //    file.print(currMillis);
  //    file.print(":     ");
  //    file.print(runMode);
  //    file.print("     ");
  //    file.print(anglex);
  //    file.print("     ");
  //    file.print(angley);
  //    file.print("     ");
  //    file.print(anglez);
  //    file.print("     ");
  //    file.print(servoanglex - centrepoint);
  //    file.print("     ");
  //    file.print(servoanglez - centrepoint);
  //    file.print("     ");
  //    file.print("alt: ");
  //    file.print("     ");
  //    file.print(Alt);
  //    file.print("     ");
  //    file.print(smoothedalt);
  //    file.print("     ");
  //    file.print(vSpeed);
  //    file.print("     ");
  //    file.print(Vin);
  //    file.print("     ");
  //    file.print("accel raw readings");
  //    file.print("     ");
  //    file.print(ax);
  //    file.print("     ");
  //    file.print(ay);
  //    file.print("     ");
  //    file.print(az);
  //    file.print("     ");
  //    file.print("gyro:");
  //    file.print("     ");
  //    file.print(gx * RAD_TO_DEG);
  //    file.print("     ");
  //    file.print(gy * RAD_TO_DEG);
  //    file.print("     ");
  //    file.print(gz * RAD_TO_DEG);
  //    file.println();
  //    file.close();
  //    //delay(100);
  //  }
  //EEPROM.write(addr, alt());
  if (datalogInterval <= 0) {

  }
  else if (currMillis - oldMillis6 >= (datalogInterval / 2)) {
    if (statWrite == true) {
      //EEPROM.commit();
      //flashDatalog();
      tone(9, 700, 200);
      ledr(1);
      //data.close();
      statWrite = false;
    }
    else {
      ledr(0);
      statWrite = true;
    }
    oldMillis6 = currMillis;
  }
  //  if (addr >= EEPROM.length()) {
  //    addr = EEPROM.length();
  //    //tone(9, 1000, 250); //BEEEEEEEP
  //  }
  else {
    addr += 1;
  }
  //file.close();
  //data.close();
}

//void formatExternalFlash() {
//  FRESULT r = f_mkfs("", FM_FAT, 0, workbuf, sizeof(workbuf));
//  if (r != FR_OK) {
//    Serial.print("Error, f_mkfs failed with error code: "); Serial.println(r, DEC);
//    while (1) yield();
//  }
//  r = f_mount(&elmchamFatfs, "0:", 1);
//  if (r != FR_OK) {
//    Serial.print("Error, f_mount failed with error code: "); Serial.println(r, DEC);
//    while (1) yield();
//  }
//  // Setting label
//  Serial.println("Setting disk label to: " DISK_LABEL);
//  r = f_setlabel(DISK_LABEL);
//  if (r != FR_OK) {
//    Serial.print("Error, f_setlabel failed with error code: "); Serial.println(r, DEC);
//    while (1) yield();
//  }
//  // unmount
//  f_unmount("0:");
//  // sync to make sure all data is written to flash
//  spiflash.syncBlocks();
//  Serial.println("Formatted flash!");
//  if (!fatfs.begin(&spiflash)) {
//    Serial.println("Error, failed to mount newly formatted filesystem!");
//    while (1) delay(1);
//  }
//  Serial.println("Flash chip successfully formatted with new empty filesystem!");
//}

void wipe() { //for wiping the entire EEPROM
  while (addr <= EEPROM.length()) {
    int writeNum;
    if (writeNum >= 254) {
      writeNum = 0;
    }
    EEPROM.write(addr, writeNum);
    writeNum += 1;
    addr += 1;
  }
  if (addr >= EEPROM.length()) {
    Serial.println("EEPROM Wiped! rebooting...");
    EEPROM.commit();
    EEPROM.end();
    delay(2000);
    watchdog_enable(1, 1);
    while (1);
  }
}

void save() {
  EEPROM.commit();
  Serial.println("saved");
}

void set(String serialInput) {
  int writeAddr;
  int writeContent;
  int operation;
  writeAddr = Serial.parseInt();
  operation = Serial.read();
  writeContent = Serial.parseInt();
  EEPROM.write(writeAddr, writeContent);
  Serial.print(writeAddr);
  Serial.print(" ");
  Serial.print(operation);
  Serial.print(" ");
  Serial.print(writeContent);
  Serial.println();
}

float vin() {
  return ((analogRead(26) / 255) * 10);
}

//void estimateAngles(float ax, float ay, float az, float gx, float gy, float gz) {
//
//  // Update the Madgwick filter with the latest sensor readings
//  filter.updateIMU(gx, gy, gz, ax, ay, az);
//
//  pitch = (filter.getPitch() * (180 / PI));
//  yaw = (filter.getYaw() * (180 / PI));
//  roll = (filter.getRoll() * (180 / PI));
//
//  anglex = pitch;
//  angley = yaw;
//  anglez = roll;
//}

//float baroVel() {
//  float alt1 = smoothedAlt();
//  static float oldAlt = alt1;
//  static unsigned long oldMillis = millis();
//
//  float barometricVelocity = (alt1 - oldAlt) / ((millis() - oldMillis) / 1000.0f);
//
//  oldAlt = alt1;
//  oldMillis = millis();
//
//  return barometricVelocity;
//}

//float smoothedAlt() {
//  float NUM_SAMPLES = 10;
//  float alt_sum = 0.0;
//
//  for (int i = 0; i < NUM_SAMPLES; i++) {
//    alt_sum += alt();
//    delay(10);
//  }
//
//  return alt_sum / NUM_SAMPLES;
//}

float baroVel() {
  static float oldAlt = smoothedAlt();
  static float oldBaroVel = 0.0; // Initialize old velocity to zero
  //static unsigned long oldMillis = millis();

  float alpha = 0.01;  // Low pass filter alpha value
  float dt = (millis() - oldMillis) / 1000.0f;  // Time since last update

  float alt1 = smoothedAlt();
  float barometricVelocity = (alt1 - oldAlt) / dt;

  // Apply low pass filter to the velocity
  barometricVelocity = alpha * barometricVelocity + (1 - alpha) * oldBaroVel;

  oldAlt = alt1;
  oldBaroVel = barometricVelocity;
  oldMillis = millis();

  return barometricVelocity;
}

float smoothedAlt() {
  float ALPHA = 0.01;
  float currentAlt = alt();
  static float previousAlt = 0.0;
  float filteredAlt;

  filteredAlt = ALPHA * currentAlt + (1 - ALPHA) * previousAlt;
  previousAlt = filteredAlt;

  return filteredAlt;
}

//void estimateLocation() {
//  // Calculate the time difference since the previous iteration
//  unsigned long currentTime = millis();
//  float dt = (currentTime - prevTime) / 1000.0;    // Convert to seconds
//  prevTime = currentTime;
//
//  // Calculate the distance traveled by the robot during the time step
//  float distance = v * dt;
//
//  // Calculate the change in orientation of the robot during the time step
//  float dtheta = omega * dt;
//
//  // Calculate the x and y components of the robot's displacement
//  float dx = distance * cos(theta + dtheta / 2.0);
//  float dy = distance * sin(theta + dtheta / 2.0);
//
//  // Update the robot's position and orientation
//  x += dx;
//  y += dy;
//  theta += dtheta;
//
//  // Calculate the forward velocity and angular velocity of the robot
//  v = RADIUS * (anglex + angley) / 2.0;
//  omega = RADIUS * (angley - anglex) / 0.2;
//}

//void estimateAngles(){
//  float ACCEL_SCALE_FACTOR = 9.81;
//  float accel_x = accel.getAccelX_mss() / ACCEL_SCALE_FACTOR;
//  float accel_y = accel.getAccelY_mss() / ACCEL_SCALE_FACTOR;
//  float accel_z = accel.getAccelZ_mss() / ACCEL_SCALE_FACTOR;
//  //float yaw = atan2(-accel_x, sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * 180 / PI;
//  float yaw = (atan2(accel_x, accel_z) * 180 / PI) - 90;
//  float pitch = (atan2(accel_x, sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * 180 / M_PI) - 90;
//  //float roll = atan2(accel_z, sqrt(pow(accel_x, 2) + pow(accel_y, 2))) * 180 / PI;
//
//  Serial.print(pitch);
//  Serial.print("     ");
//  //Serial.print(roll);
//  //Serial.print("     ");
//  Serial.print(yaw);
//  Serial.print("     ");
//
//  if (lockGyro == true){
//    anglex = pitch;
//    angley = 0;
//    anglez = yaw;
//  }
//  else{
//    anglex = (anglex + (gyro.getGyroX_rads() * (M_PI / 180) * ((4078/71) * (cycleTime / 10))));
//    angley = (angley + (gyro.getGyroY_rads() * (M_PI / 180) * ((4078/71) * (cycleTime / 10))));
//    anglez = (anglez + (gyro.getGyroZ_rads() * (M_PI / 180) * ((4078/71) * (cycleTime / 10))));
//  }
//}

//void estimateAngles(float ax, float ay, float az, float gx, float gy, float gz, float& roll, float& pitch, float& yaw) {
//  float deltat = cycleTime;
//  float q1, q2, q3, q4;
//  const float q1q1 = q1 * q1;
//  const float q1q2 = q1 * q2;
//  const float q1q3 = q1 * q3;
//  const float q1q4 = q1 * q4;
//  const float q2q2 = q2 * q2;
//  const float q2q3 = q2 * q3;
//  const float q2q4 = q2 * q4;
//  const float q3q3 = q3 * q3;
//  const float q3q4 = q3 * q4;
//  const float q4q4 = q4 * q4;
//
//  // Normalise accelerometer measurement
//  const float norm = sqrt(ax * ax + ay * ay + az * az);
//  ax /= norm;
//  ay /= norm;
//  az /= norm;
//
//  // Gradient decent algorithm corrective step
//  const float s1 = 2.0f * (q2q4 - q1q3) - ax;
//  const float s2 = 2.0f * (q1q2 + q3q4) - ay;
//  const float s3 = 2.0f * (0.5f - q2q2 - q3q3) - az;
//  const float v1 = 2.0f * (q2q4 - q1q3);
//  const float v2 = 2.0f * (q1q2 + q3q4);
//  const float v3 = q1q1 - q2q2 - q3q3 + q4q4;
//
//  // Compute angular acceleration
//  const float axg = gx + s1;
//  const float ayg = gy + s2;
//  const float azg = gz + s3;
//
//  // Integrate angular acceleration to obtain angular velocity
//  q1 += (-q2 * axg - q3 * ayg - q4 * azg) * (0.5f * deltat);
//  q2 += (q1 * axg + q3 * azg - q4 * ayg) * (0.5f * deltat);
//  q3 += (q1 * ayg - q2 * azg + q4 * axg) * (0.5f * deltat);
//  q4 += (q1 * azg + q2 * ayg - q3 * axg) * (0.5f * deltat);
//
//  // Normalize quaternion
//  const float normq = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
//  q1 /= normq;
//  q2 /= normq;
//  q3 /= normq;
//  q4 /= normq;
//
//  // Calculate pitch, yaw, and roll
//  pitch = atan2(2.0f * (q2q3 + q1q4), q1q1 - q2q2 - q3q3 + q4q4);
//  yaw = asin(-2.0f * (q2q4 - q1q3));
//  roll = atan2(2.0f * (q1q2 + q3q4), q1q1 + q2q2 - q3q3 - q4q4);
//
// // Convert from radians to degrees
//  pitch *= 180.0f / PI;
//  yaw *= 180.0f / PI;
//  roll *= 180.0f / PI;
//
//  Serial.print(pitch);
//  Serial.print("     ");
//  Serial.print(yaw);
//  Serial.print("     ");
//  Serial.print(roll);
//  Serial.print("     ");
//}

//--------------------------------------------------------------------+
// fatfs diskio
//--------------------------------------------------------------------+

//extern "C"
//{
//
//  DSTATUS disk_status ( BYTE pdrv )
//  {
//    (void) pdrv;
//    return 0;
//  }
//
//  DSTATUS disk_initialize ( BYTE pdrv )
//  {
//    (void) pdrv;
//    return 0;
//  }
//
//  DRESULT disk_read (
//    BYTE pdrv,    /* Physical drive nmuber to identify the drive */
//    BYTE *buff,   /* Data buffer to store read data */
//    DWORD sector, /* Start sector in LBA */
//    UINT count    /* Number of sectors to read */
//  )
//  {
//    (void) pdrv;
//    return spiflash.readBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
//  }
//
//  DRESULT disk_write (
//    BYTE pdrv,      /* Physical drive nmuber to identify the drive */
//    const BYTE *buff, /* Data to be written */
//    DWORD sector,   /* Start sector in LBA */
//    UINT count      /* Number of sectors to write */
//  )
//  {
//    (void) pdrv;
//    return spiflash.writeBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
//  }
//
//  DRESULT disk_ioctl (
//    BYTE pdrv,    /* Physical drive nmuber (0..) */
//    BYTE cmd,   /* Control code */
//    void *buff    /* Buffer to send/receive control data */
//  )
//  {
//    (void) pdrv;
//
//    switch ( cmd )
//    {
//      case CTRL_SYNC:
//        spiflash.syncBlocks();
//        return RES_OK;
//
//      case GET_SECTOR_COUNT:
//        *((DWORD*) buff) = spiflash.size() / 512;
//        return RES_OK;
//
//      case GET_SECTOR_SIZE:
//        *((WORD*) buff) = 512;
//        return RES_OK;
//
//      case GET_BLOCK_SIZE:
//        *((DWORD*) buff) = 8;    // erase block size in units of sector size
//        return RES_OK;
//
//      default:
//        return RES_PARERR;
//    }
//  }
//
//}
