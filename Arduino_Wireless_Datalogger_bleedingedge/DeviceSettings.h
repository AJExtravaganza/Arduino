#ifndef DEVICESETTINGS
#define DEVICESETTINGS

struct DeviceSettings {
  byte deviceID;

  DeviceSettings() : deviceID(0){}
  DeviceSettings(int deviceID) : deviceID(byte(deviceID)) {}

  void read() {*this = EEPROM.read(0);}
  void write() {EEPROM.write(0, this);}

  
};

#endif