#include "DeviceSettings.h"
#include "EEPROMAnything.h"
#include "printf.h"

DeviceSettings settings(0);

void setup() {
  Serial.begin(57600);
  printf_begin();
  printf ("Device setup. 10 seconds until loop.\n\n");
  delay(10000);
 
}

void loop() {
  //printf ("Device started.\n\n");



  settings.deviceID = 0;
  settings.write();
  settings.deviceID = 1;
  settings.read();
  printf("Device is now a Base with ID %u.\n\n", settings.deviceID);
  delay(5000);

  settings.deviceID = 1;
  settings.write();
  settings.deviceID = 0;
  settings.read();
  printf("Device is now a Sat with ID %u.\n\n", settings.deviceID);
  delay(5000);

  settings.deviceID = 2;
  settings.write();
  settings.deviceID = 0;
  settings.read();
  printf("Device is now a Sat with ID %u.\n\n", settings.deviceID);
  delay(5000);

  settings.deviceID = 3;
  settings.write();
  settings.deviceID = 0;
  settings.read();
  printf("Device is now a Sat with ID %u.\n\n", settings.deviceID);
  delay(5000);
  

}
