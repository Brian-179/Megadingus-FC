//#include "Arduino.h"
//#include "global_variables.h"
//
//#include "EEPROM.h"
//#include <LittleFS.h>
//#include <SD.h>
//
//void wipe() { //for wiping the entire EEPROM
//  while (addr <= EEPROM.length()) {
//    int writeNum;
//    if (writeNum >= 254) {
//      writeNum = 0;
//    }
//    EEPROM.write(addr, writeNum);
//    writeNum += 1;
//    addr += 1;
//  }
//  if (addr >= EEPROM.length()) {
//    Serial.println("EEPROM Wiped! rebooting...");
//    EEPROM.commit();
//    EEPROM.end();
//    delay(2000);
//    watchdog_enable(1, 1);
//    while (1);
//  }
//}
//
//void save() {
//  EEPROM.commit();
//  Serial.println("saved");
//}
//
//void set(String serialInput) {
//  int writeAddr;
//  int writeContent;
//  int operation;
//  writeAddr = Serial.parseInt();
//  operation = Serial.read();
//  writeContent = Serial.parseInt();
//  EEPROM.write(writeAddr, writeContent);
//  Serial.print(writeAddr);
//  Serial.print(" ");
//  Serial.print(operation);
//  Serial.print(" ");
//  Serial.print(writeContent);
//  Serial.println();
//}
//
//float vin() {
//  return ((analogRead(26) / 255) * 10);
//}
//
//void datalog() {
//  if (datalogInterval <= 0) {
//
//  }
//  else if (currMillis - oldMillis6 >= (datalogInterval / 2)) {
//    if (statWrite == true) {
//      //EEPROM.commit();
//      //flashDatalog();
//      tone(9, 700, 200);
//      ledr(1);
//      //data.close();
//      statWrite = false;
//    }
//    else {
//      ledr(0);
//      statWrite = true;
//    }
//    oldMillis6 = currMillis;
//  }
//  //  if (addr >= EEPROM.length()) {
//  //    addr = EEPROM.length();
//  //    //tone(9, 1000, 250); //BEEEEEEEP
//  //  }
//  else {
//    addr += 1;
//  }
//  //file.close();
//  //data.close();
//}
//
//
//void SDInfo() {
//  //  if (!volume.init(card)){
//  //    Serial.println("sd volume error");
//  //  }
//  //  uint32_t volumeSize;
//  //  volumeSize = volume.blocksPerCluster();
//  //  volumeSize *= volume.clusterCount();
//  //  volumeSize /= 2;
//  //  Serial.print("volume size in K:");
//  //  Serial.println(volumeSize);
//  //  Serial.println("=");
//  //  Serial.print(volumeSize / 1024);
//  //  Serial.println("MB");
//  //  Serial.println("=");
//  //  Serial.print((volumeSize / 1024) / 1024);
//  //  Serial.println("GB");
//  //  root.openRoot(volume);
//  //  root.ls(LS_R | LS_DATE | LS_SIZE);
//  //  root.close();
//}
//
////void eepromtoSD() {
////  int oldRunMode = runMode;
////  runMode = -2; //temporarily put it into CLI mode, so the cores aren't constantly fighting over the serial terminal
////  static int readaddr = 0;
////  while (readaddr <= EEPROM.length()) {
////    delay(5);
////    File file = SD.open("eeprom.txt", FILE_WRITE);
////    file.print(readaddr);
////    file.print(": ");
////    file.println(EEPROM.read(readaddr));
////    Serial.println(readaddr);
////    readaddr += 1;
////    file.close();
////  }
////  runMode = oldRunMode;
////}
//
//void dump() {
//  int oldRunMode = runMode;
//  while (readAddr <= EEPROM.length()) {
//    Serial.print(readAddr);
//    Serial.print("     ");
//    Serial.println(EEPROM.read(readAddr));
//    readAddr += 1;
//    delay(1); //delay to help with printing
//  }
//  if (readAddr >= EEPROM.length()) {
//    Serial.print("that's all ");
//    Serial.print(EEPROM.length());
//    Serial.println(" bytes of the eeprom dumped!");
//    readAddr = 0;
//    ledr(1);
//    ledg(1);
//    ledb(0);
//    delay(2000);
//    runMode = oldRunMode;
//  }
//}
//
//void SDDump() {
//  int oldRunMode = runMode;
//  File file = SD.open("datalog.txt", FILE_READ);
//  if (file) {
//    while (file.available()) {
//      Serial.write(file.read());
//    }
//    file.close();
//  }
//  else {
//    Serial.println("sd file error");
//  }
//}
//
//
//void processSerial(String serialInput) {
//  serialInput.trim();
//  if (serialInput == "dump") {
//    dump();
//  }
//  //    if (serialInput == "dumpFlash"){
//  //      File data = LittleFS.open("/data.txt","r");
//  //      while (data.available()){
//  //        Serial.write(data.read());
//  //      }
//  //    }
//  //    if (serialInput == "clearSD") {
//  //      SD.rmdir("/");
//  //      if (SD.rmdir("/") == 1) {
//  //        Serial.println("done");
//  //      }
//  //      else {
//  //        Serial.println("error of some sort, im sorry that its not helpful");
//  //      }
//  //    }
//  //    if (serialInput == "formatSD") {
//  //      SDFS.format();
//  //      Serial.println("done");
//  //    }
//  //    if (serialInput == "formatFlash"){
//  //      LittleFS.format();
//  //      Serial.println("formatted flash!");
//  //    }
//  if (serialInput == "SDInfo") {
//    SDInfo();
//  }
//  if (serialInput == "dump_telem") {
//    readAddr = startAddr;
//    dump();
//  }
//  if ((serialInput == "clr_eeprom") or (serialInput == "eeprom_clr")) {
//    addr = 0;
//    wipe();
//  }
//  if ((serialInput == "clr_datalog") or (serialInput == "datalog_clr")) {
//    addr = startAddr;
//    wipe();
//  }
//  if (serialInput == "eepromtoSD") {
//    //eepromtoSD();
//  }
//
//  if (serialInput == "reboot") {
//    watchdog_enable(1, 1);
//    while (1);
//  }
//  if (serialInput == "save") {
//    save();
//  }
//  if (serialInput == "set") {
//    set(serialInput);
//  }
//  //    if (serialInput == "defaults") {
//  //      EEPROM.write(0, defaultP);
//  //      EEPROM.write(1, defaultI);
//  //      EEPROM.write(2, defaultD);
//  //      EEPROM.write(3, defaultDelay);
//  //      Serial.println("reset settings to defaults");
//  //    }
//  if (serialInput == "getpids") {
//    Serial.println(Kp);
//    Serial.println(Ki);
//    Serial.println(Kd);
//    Serial.println(delayPeriod);
//  }
//  if (serialInput == "cli") {
//    runMode = -2;
//    ledr(1);
//    ledg(1);
//    ledb(1);
//    Serial.println("CLI mode!");
//    Serial.println("reboot to exit");
//  }
//  if (serialInput == "landed") {
//    runMode = 4;
//  }
//  if (serialInput == "waiting") {
//    runMode = 0;
//  }
//  if (serialInput == "poweredAscent") {
//    runMode = 1;
//  }
//  if (serialInput == "coast") {
//    runMode = 2;
//  }
//  if (serialInput == "descent") {
//    runMode = 3;
//  }
//  if (serialInput == "testAllPyros") {
//    Serial.println("Pyros will be held ON until otherwise");
//    digitalWrite(pyro1, HIGH);
//    digitalWrite(pyro2, HIGH);
//    digitalWrite(pyro3, HIGH);
//  }
//  if (serialInput == "allPyrosOff") {
//    Serial.println("All pyro channels OFF");
//    digitalWrite(pyro1, LOW);
//    digitalWrite(pyro2, LOW);
//    digitalWrite(pyro3, LOW);
//  }
//  if (serialInput == "pyro1") {
//    //TVC.pyro1();
//    ledr(1);
//    ledg(0);
//    ledb(0);
//    digitalWrite(pyro1, HIGH);
//    delay(250);
//    digitalWrite(pyro1, LOW);
//    ledr(0);
//    ledg(1);
//    ledb(1);
//    Serial.println("fired pyro channel 1 (hopefully)");
//  }
//  if (serialInput == "pyro2") {
//    TVC.pyro2();
//  }
//  if (serialInput == "pyro3") {
//    TVC.pyro3();
//  }
//  if (serialInput == "servox") {
//    servox.write(centrepointX + servoLimit);
//    delay(100);
//    servox.write(centrepointX - servoLimit);
//    delay(100);
//    servox.write(centrepointX);
//  }
//  if (serialInput == "servoy") {
//    servoy.write(centrepointY + servoLimit);
//    delay(100);
//    servoy.write(centrepointY - servoLimit);
//    delay(100);
//    servoy.write(centrepointY);
//  }
//  if (serialInput == "dumpFlash") {
//    int oldrunmode = runMode;
//    runMode = -2;
//    File data = LittleFS.open("/data.txt", "r");
//    Serial.println("dumping flash:");
//    while (data.available()) {
//      Serial.write(data.read());
//    }
//    data.close();
//    runMode = oldrunmode;
//    //serialInput = "";
//  }
//  if (serialInput == "defaults") {
//    LittleFS.remove("/config.txt");
//    File file = LittleFS.open("/config.txt", "a");
//    file.println(defaultP);
//    file.println(defaultI);
//    file.println(defaultD);
//    file.println(defaultDelay);
//    file.close();
//    rp2040.reboot();
//  }
//  if ((serialInput == "deleteTelem") or (serialInput == "clearTelem")) {
//    if (LittleFS.exists("/config.txt")) {
//      LittleFS.remove("/config.txt");
//      Serial.println("done");
//    }
//    else {
//      Serial.println("the config file for some unhelpful reason does not appear to exist");
//    }
//    //serialInput = "";
//  }
//  if ((serialInput == "dumpConfig") or (serialInput == "getConfig")) {
//    int oldrunmode = runMode;
//    runMode = -2;
//    File file = LittleFS.open("config.txt", "r");
//    Serial.println("dumping config from flash");
//    while (file.available()) {
//      Serial.write(file.read());
//    }
//    file.close();
//    runMode = oldrunmode;
//    //serialInput = "";
//  }
//  //  if (BOOTSEL) {
//  //    buz(1);
//  //    runMode = 4;
//  //  }
//  //  if (!BOOTSEL) {
//  //    buz(0);
//  //  }
//  if (serialInput == "flashtoSD") {
//    flashtoSD();
//    //serialInput = "";
//  }
//  if (serialInput == "SDtoFlash" or serialInput == "SDtoflash") {
//    SDtoflash();
//  }
//  if ((serialInput == "SDDump") or (serialInput == "dumpSD")) {
//    SDDump();
//    //serialInput = "";
//  }
//  if (serialInput == "formatExternalFlash") {
//    //formatExternalFlash();
//    //serialInput = "";
//  }
//  //  if (serialInput == "flashtoSD") {
//  //    int oldrunmode = runMode;
//  //    runMode = -2;
//  //    File data = LittleFS.open("/data.txt", "r");
//  //    File file = SD.open("flash.txt", FILE_WRITE);
//  //    while (data.available()) {
//  //      file.write(data.read());
//  //      //rp2040.fifo.push(data.read());
//  //      //fifo = data.read();
//  //    }
//  //  data.close();
//  //  file.close();
//  //  Serial.println("Copied all data from internal Flash to MicroSD card");
//  //  runMode = oldrunmode;
//  //}
//  if (serialInput == "formatFlash") {
//    LittleFS.format();
//    Serial.println("formatted flash!");
//    //serialInput = "";
//  }
//  if (serialInput == "clearFlashDatalog") {
//    LittleFS.remove("/data.txt");
//    Serial.println("done");
//  }
//  if (serialInput == "wipeFlash") {
//    LittleFS.rmdir("/");
//    Serial.println("done");
//  }
//  //    if (serialInput == "clearExternalFlash") {
//  //      fatfs.rmdir("/");
//  //      Serial.println("done");
//  //      //serialInput = "";
//  //    }
//  //  if (serialInput == "formatExternalFlash") {
//  //    // Make filesystem.
//  //    FRESULT r = f_mkfs("", FM_FAT, 0, workbuf, sizeof(workbuf));
//  //    if (r != FR_OK) {
//  //      Serial.print("Error, f_mkfs failed with error code: "); Serial.println(r, DEC);
//  //      while (1) yield();
//  //    }
//  //    // mount to set disk label
//  //    r = f_mount(&elmchamFatfs, "0:", 1);
//  //    if (r != FR_OK) {
//  //      Serial.print("Error, f_mount failed with error code: "); Serial.println(r, DEC);
//  //      while (1) yield();
//  //    }
//  //    // Setting label
//  //    Serial.println("Setting disk label to: " DISK_LABEL);
//  //    r = f_setlabel(DISK_LABEL);
//  //    if (r != FR_OK) {
//  //      Serial.print("Error, f_setlabel failed with error code: "); Serial.println(r, DEC);
//  //      while (1) yield();
//  //    }
//  //    // unmount
//  //    f_unmount("0:");
//  //    // sync to make sure all data is written to flash
//  //    flash.syncBlocks();
//  //    Serial.println("Formatted flash!");
//  //    if (!fatfs.begin(&flash)) {
//  //      Serial.println("Error, failed to mount newly formatted filesystem!");
//  //      while (1) delay(1);
//  //    }
//  //  }
//  //    if (serialInput == "dumpExternalFlash") {
//  //      Serial.println("dumping external flash");
//  //      File32 flash = fatfs.open("datalog.txt", FILE_READ);
//  //      while (flash.available()) {
//  //        Serial.write(flash.read());
//  //      }
//  //      //serialInput = "";
//  //    }
//  if (serialInput == "gimbalTest") {
//    Serial.println("going into gimbalTest mode, adding accel into angles");
//    runMode = 69;
//  }
//  if (serialInput == "countdown") {
//    Serial.println("I will count down from 10, then go into poweredAscent mode");
//    runMode = -3;
//  }
//  if (serialInput == "calibrateESC") {
//    Serial.println("make sure the esc on the servo3 connector is unplugged from VBAT");
//    prop.write(0);
//    delay(2000);
//    Serial.println("plug in the ESC and wait");
//    prop.write(180);
//    delay(15000);
//    prop.write(0);
//  }
//  if (serialInput == "testProp") {
//    Serial.println("testing prop");
//    prop.write(10);
//    delay(5000);
//    prop.write(0);
//  }
//  if (serialInput == "testPropInf") {
//    Serial.println("the prop will run until stopProp is run");
//    prop.write(10);
//  }
//  if (serialInput == "stopProp") {
//    Serial.println("stopping prop");
//    prop.write(0);
//  }
//  if (serialInput == "propSpeedTest") {
//    Serial.println("prop speed will be ramped up from 0 to max and back");
//    int propSpeed = 0;
//    while (propSpeed <= 50) {
//      propSpeed += 1;
//      prop.write(propSpeed);
//      delay(30);
//    }
//    while (propSpeed >= 0) {
//      propSpeed -= 1;
//      //        if (propSpeed <= 0){
//      //          propSpeed = 0;
//      //        }
//      prop.write(propSpeed);
//      delay(30);
//    }
//    prop.write(0);
//  }
//  //serialInput = "";
//  if (serialInput == "formatSD") {
//    SDFS.format();
//    Serial.println("done");
//  }
//  if (serialInput == "wipeSD") {
//    SD.remove("datalog.txt");
//    SD.rmdir("/");
//    Serial.println("done");
//  }
//  if ((serialInput == "autoClicker") or (serialInput == "autoclicker")) {
//    Mouse.begin();
//    while (1) {
//      Mouse.click();
//    }
//  }
//  //  if (serialInput == "eepromtoSD"){
//  //    eepromtoSD();
//  //  }
//  serialInput = ""; //flush the buffer
//}
