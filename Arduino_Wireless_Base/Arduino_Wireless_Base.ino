
  //// Use these if compiling from Arduino IDE ////
#include <EEPROM.h>
#include <Wire.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "DeviceSettings.h"
#include "Transmission.h"
#include "Satellite.h"
#include "BME280.h"
#include "mechElec.h"


  //// These are for CLion ////
//#include <cstdint>
//#include "../libraries/RF24-master/RF24.h"
//#include "../libraries/RF24-master/nRF24L01.h"
//#include "../libraries/PuLogger/DeviceSettings.h"
//#include "../libraries/PuLogger/Transmission.h"
//#include "../libraries/PuLogger/Satellite.h"
//#include "../libraries/PuLogger/mechElec.h"

const byte SATELLITES = 1;
const byte DEVICES = SATELLITES + 1;
unsigned long int lastCheckedIn = 0; // Holds last time satellite contacted base (by successfully sending a transmission)  Used by sats only.

  ////Definable variables that determine transmission frequency////
const unsigned long int CHECKINPERIOD = 1000UL *  10UL;// * 60UL * 15UL; // Check in once every fifteen minutes
const byte DEADMANTHRESHOLD = 50; //Exceeding CHECKINPERIOD by 50% will cause deadman alarm

  //// Hysteresis values in deci-units
const byte TEMPHYS = 1;  
const byte HUMHYS = 5; 

  ////These are Tingle's magic derived constants.  Do not touch them.////
const unsigned long int DEADMANPERIOD = CHECKINPERIOD * (100UL + static_cast<unsigned long int>(DEADMANTHRESHOLD)) / 100UL;
const unsigned long int SENSORPOLLPERIOD = CHECKINPERIOD / 10UL - 1UL; //Must be <DEADMANTHRESHOLD% of DEADMANPERIOD
const unsigned long int SATELLITELOOPPERIOD = SENSORPOLLPERIOD / 3UL; // Must be <=SENSORPOLLPERIOD

  //// Global variable to store address of current sensor to poll
bool hasAdditionalSensor = true; //fixme change back to false when EEPROM stuff is implemented;
uint8_t sensorAddress[2] = {0x77, 0x76};

  ////Satellite objects for base station
Satellite satellites[DEVICES];

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 with particular payload size
RF24 radio(9, 10);
const byte MAXTRANSMITATTEMPTS = 50;
const byte RADIOPAYLOADSIZE = 10; //Must be >= sizeof(Transmission)

// Radio pipe addresses for 6 nodes to communicate.
const uint64_t PROGMEM pipes[6] = {0xF0F0F0F0D0LL, 0xF0F0F0F0D1LL, 0xF0F0F0F0D2LL, 0xF0F0F0F0D3LL, 0xF0F0F0F0D4LL, 0xF0F0F0F0D5LL};

  //// Role management

// The various roles supported by this sketch
typedef enum {
  role_base = 1, role_satellite
} role_e;

// The debug-friendly names of those roles
const char* role_friendly_name[] = {"invalid", "base", "satellite"};


// Declare role and ID
int deviceID = -1;
role_e role = role_base;

//// Has particular Satellite gone >DEADMANPERIOD without checking in?
bool deviceFailure(int deviceID) {

    if (static_cast<unsigned long int>(millis() - satellites[deviceID].lastTransmission) > DEADMANPERIOD && millis() > SATELLITELOOPPERIOD) { 
      satellites[deviceID].deviceUp = false;
    }
    else {
      satellites[deviceID].deviceUp = true;
    }

    return !satellites[deviceID].deviceUp;
}

  //// Check for new device time-outs and send failure any failure status to gui.
bool checkDeviceTimeout(int deviceID, bool &deviceStatusUnknown) {
  bool wasLive = satellites[deviceID].deviceUp;

  if (deviceFailure(deviceID)) { //Check for devices that haven't touched base recently.  If such exists,
    if (wasLive || deviceStatusUnknown) {  // If state has changed, or satellite hasn't checked in in the first DEADMANPERIOD of runtime
      printf(">STS;%i;0;%lu\n", deviceID, (millis() / 1000)); // Set device status DOWN.
     deviceStatusUnknown = false;
    }
    else {
      //printf("Device still down.\n");
    }
   
    beep();
  }
}

void clearAllAlarms() {
  for (int i = 1; i < DEVICES; i++) {
    satellites[i].clearAlarms();
  }
}

  //// Translate Transmission data update to raw values
void update(Transmission received) {
  satellites[received.deviceID].update(0, received.getRawTemp(0), received.getRawHum(0), millis());
  //printf("updating sensor 0 with %i, %i\n", received.getRawTemp(0), received.getRawHum(0)); //debug

 if (satellites[received.deviceID].hasAdditionalSensor || true) { //debug forcing conditional for debugging
   satellites[received.deviceID].update(1, received.getRawTemp(1), received.getRawHum(1), millis());
  //printf("updating sensor 1 with %i, %i\n", received.getRawTemp(1), received.getRawHum(1)); //debug
 }
}

void setup(void) {

 //// Set buzzer, fan and heater pinModes
  pinMode(BUZZERPIN, OUTPUT);
  pinMode(FANPIN, OUTPUT);
  pinMode(HEATPIN, OUTPUT);

  
    //// Start the serial service.
  Serial.begin(57600);

    ////Initialise Satellite object deviceIDs
  for (int i = 1; i <= SATELLITES; i++) { 
    satellites[i].deviceID = i;
  }
  
  //fixme set hasAdditionalSensor = true.  later this will be done automatically by checking sensor for valid read values (or stored on eeprom?)
  if (deviceID == 1) {
    hasAdditionalSensor = true;
  }
  
  //// Load settings from EEPROM and assign role
  DeviceSettings settings;
  settings.read();
  deviceID = settings.deviceID;
  role = settings.deviceID == 0 ? role_base : role_satellite;
  
  //// BME280 and I2C Setup
  
  uint8_t osrs_t = 1;             //Temperature oversampling x 1
  uint8_t osrs_p = 1;             //Pressure oversampling x 1
  uint8_t osrs_h = 1;             //Humidity oversampling x 1
  uint8_t mode = 3;               //Normal mode
  uint8_t t_sb = 5;               //Tstandby 1000ms
  uint8_t filter = 0;             //Filter off 
  uint8_t spi3w_en = 0;           //3-wire SPI Disable

  uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
  uint8_t config_reg = (t_sb << 5) | (filter << 2) | spi3w_en;
  uint8_t ctrl_hum_reg = osrs_h;

  Wire.begin();

  writeReg(0xF2, ctrl_hum_reg, sensorAddress[0]);
  writeReg(0xF4, ctrl_meas_reg, sensorAddress[0]);
  writeReg(0xF5, config_reg, sensorAddress[0]);
  readTrim(sensorAddress[0]);

  if (hasAdditionalSensor) {
    writeReg(0xF2, ctrl_hum_reg, sensorAddress[1]);
    writeReg(0xF4, ctrl_meas_reg, sensorAddress[1]);
    writeReg(0xF5, config_reg, sensorAddress[1]);
    readTrim(sensorAddress[1]);
  }

  printf_begin();
  //printf("ROLE: %s with ID %i\n\r", role_friendly_name[role], settings.deviceID);

    ////Radio Configuration
  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15, 15);

  // optionally, reduce the payload size.  seems to improve reliability
  radio.setPayloadSize(RADIOPAYLOADSIZE);

  radio.setDataRate(RF24_250KBPS);
  
    //// Set radio pre-amp level
  radio.setPALevel(RF24_PA_MAX);

    //// Open pipes for Transmission send/receive
  if (role == role_base) {
    radio.openReadingPipe(0, pipes[0]); //Open channel to receive comms from all satellites
  }
  else {
    radio.openWritingPipe(pipes[0]); //All satellites send data to base on channel 0
    radio.openReadingPipe(1, pipes[0]); //Monitor base rx channel to check it's ok to tx
    radio.openReadingPipe(1, pipes[deviceID]); //Each satellite listens for commands on its matching channel
  }

  radio.startListening();

  //Commented out for Qt QString compatibility, which can't handle tabs in QString.left()
  //radio.printDetails(); //Outputs detailed information on radio unit and settings

  beep(); // Indicate successful startup (ie no serial lockup)
}


void loop(void) {
  //// Base role; Device connected to computer to receive transmissions from the satellite
  if (role == role_base) {

    //printf("Size of Transmission object is %i compared to maximal nRF24L01+ packet size of %i.\n", sizeof(test), RADIOPAYLOADSIZE);

    bool deviceStatusUnknown[DEVICES]; // Tracks whether we've determined a device status of a particular satellite since bas boot
    for (int i = 1; i <= SATELLITES; i++) {
      deviceStatusUnknown[i] = true;
      //satellites[i].hasAdditionalSensor = true; // debug Breaks the DAT xmit... wtf
      }
    
    
    while (role == role_base) {
           
      Transmission received(-1, 0, 0, 0, 0);
      
        //// Check each device's status
      for (int i = 1; i <= SATELLITES; i++) {
        if (checkDeviceTimeout(i, deviceStatusUnknown[i])) {
          beep();
        }
      }
    
        //// Process incoming Transmissions
      if (radio.available()) { // If an incoming transmission is pending
        
        radio.read(&received, sizeof(received)); // Read it
        //printf("Just received ");
        // received.printCSV();
        
        if (satellites[received.deviceID].deviceUp == false || deviceStatusUnknown[received.deviceID]) {
          printf(">STS;%i;1;%lu;\n", received.deviceID, (millis() / 1000));
          deviceStatusUnknown[received.deviceID] = false;
          
        }
        satellites[received.deviceID].deviceUp = true;
        satellites[received.deviceID].lastTransmission = millis(); // Update time record of most recent  transmission for deadman purposes.
        
        update(received); // Update the relevant Satellite object with the new values
        
        printf(">DAT;%i;", received.deviceID); // Output CSV with ID,
        printf("%lu;", (millis() / 1000)); // timestamp (seconds since base boot),
        received.printCSV(); // and values.


          ////Basic alarm reporting functionality
        if (satellites[received.deviceID].tempLowAlarm) {
          //printf(">ALM;%i;LT;Low Temperature;\n", received.deviceID);
          beep();
        }
        if (satellites[received.deviceID].tempHighAlarm) {
          //printf(">ALM;%i;HT;High Temperature;\n", received.deviceID);
          beep();
        }
        if (satellites[received.deviceID].humLowAlarm) {
          //printf(">ALM;%i;LH;Low Humidity;\n", received.deviceID);
          beep();
        }
        if (satellites[received.deviceID].humHighAlarm) {
          //printf(">ALM;%i;HH;High Humidity;\n", received.deviceID);
          beep();
        }

        clearAllAlarms(); // Necessary until gui is able to xmit a command to base.  Does not reset FirstOOR timers
          
      }
        ////Delay to allow satellite to switch to receive mode (currently not useful).
      delay(20);
    }
  }
}
