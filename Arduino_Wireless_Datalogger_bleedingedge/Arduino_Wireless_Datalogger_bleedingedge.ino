//// TO DO
//// Work on implementing controls signal communication so that base station can drive outputs on the satellite
//// Sanity check for erroneous values (temp/hum should not change more than 1-2deg at a time, and value in array should be limited to this range of change
//// Remember to comment out debug for final deployment


#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "DeviceSettings.h"
#include "Transmission.h"
#include "BME280.h"


//
// Hardware configuration
//

const unsigned int SATELLITES = 2;
bool liveDevices[SATELLITES] = {1,1};
const unsigned long int DEADMANPERIOD = 1000UL * 60UL * 60UL;// * 24UL; // Check once per day
const unsigned long int SATELLITEPOLLPERIOD = 1000UL;// * 60UL * 15UL; //Satellite poll rate
unsigned long int lastDeadmanCheck = 0; // Holds last time device status was checked

const float TEMPHYS = 0.5;  // Hysteresis values
const float HUMHYS = 0.5; 

bool tempDelivered = false; //Only used for satellites.  Declared here for persistence over different loop() iterations
bool humDelivered = false;  // Move these to one of the infinite internal loops at a later date


// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 
RF24 radio(9, 10);


//
// Topology
//

// Radio pipe addresses for 6 nodes to communicate.
const uint64_t pipes[6] = {0xF0F0F0F0D0LL, 0xF0F0F0F0D1LL, 0xF0F0F0F0D2LL, 0xF0F0F0F0D3LL, 0xF0F0F0F0D4LL, 0xF0F0F0F0D5LL};


//
// Role management
//
// Set up role.  This sketch uses the same software for all the nodes
// in this system.  Doing so greatly simplifies testing.  
//

// The various roles supported by this sketch
typedef enum {
  role_base = 1, role_satellite
} role_e;

// The debug-friendly names of those roles
const char* role_friendly_name[] = {"invalid", "base", "satellite"};


// Declare role and ID
int deviceID = -1;
role_e role = role_base;

bool deadmanCheck(int deviceID) {
  bool deviceResponded = false;
  long long unsigned int startTime = millis();

  radio.stopListening();
  radio.openWritingPipe(pipes[deviceID]);
  
  while (!deviceResponded && millis() - startTime < 1000) { //Try for 1000ms
    deviceResponded = radio.write(&deviceResponded,sizeof(deviceResponded)); //Send a ping and wait for it to be read by target radio
  }
  
  liveDevices[deviceID] = deviceResponded; //Write status to array
  radio.startListening();
}

void checkForLife(bool checkedIn[]) {
  for (int i = 1; i <= SATELLITES; i++) {
    deadmanCheck(i);
  }
  lastDeadmanCheck = millis();
}


/* TROUBLE INCLUDING STRING LIBRARY FILE FOR SOME REASON
 
string millisAsHHMM(unsigned long int milliseconds) {
  string hhmm = std::to_string(milliseconds/1000/60) + "m" + std::to_string(milliseconds/1000%60) + "s";
  return hhmm
}

*/

void setup(void) {
  ////Load settings from EEPROM and assign role
  DeviceSettings settings;
  settings.read();
  deviceID = settings.deviceID;
  role = settings.deviceID == 0 ? role_base : role_satellite;
  
  ////BME280 and SPI Setup
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

  Serial.begin(57600);
  Wire.begin();

  writeReg(0xF2, ctrl_hum_reg);
  writeReg(0xF4, ctrl_meas_reg);
  writeReg(0xF5, config_reg);
  readTrim();

  printf_begin();
  printf("ROLE: %s with ID %i\n\r", role_friendly_name[role], settings.deviceID);

  //
  // Setup and configure rf radio
  //

  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15, 15);

  // optionally, reduce the payload size.  seems to
  // improve reliability
  radio.setPayloadSize(8);

  radio.setDataRate(RF24_250KBPS);
  
  // Set radio amplification level
  radio.setPALevel(RF24_PA_MAX);

  //
  // Open pipes to other nodes for communication
  //

  // This simple sketch opens two pipes for these two nodes to communicate
  // back and forth.
  // Open 'our' pipe for writing
  // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)

  if (role == role_base) {
    radio.openReadingPipe(1, pipes[0]); //Open channel to receive comms from all satellites
  }
  else {
    radio.openWritingPipe(pipes[0]); //All satellites send data to base on channel 0
    radio.openReadingPipe(1, pipes[deviceID]); //Each satellite listens for commands on its matching channel
  }

  radio.startListening();
  
  radio.printDetails(); //Outputs detailed information on radio unit and settings
}


void loop(void) {
  
  if (role == role_satellite) {  //Satellite setup
    long long unsigned int lastSatellitePoll = 0;
    double temp_act = 0.0, hum_act = 0.0;
    signed long int temp_cal;
    unsigned long int hum_cal;
    Transmission prevPayload(-1, 0.0, 0.0);

    readData();

    temp_cal = calibration_T(temp_raw);
    hum_cal = calibration_H(hum_raw);

    double del_temp_act = 0; //Change in temperature since last transmission
    double del_hum_act = 0; //Change in humidity since last transmission

    while (role == role_satellite) { //Satellite main loop
      
      if (radio.available()) { //If base is checking for life-sign
        bool basePing = radio.read(&basePing, sizeof(basePing)); //Accept the ping
      }
      
      if (millis() - lastSatellitePoll > SATELLITEPOLLPERIOD) { //If it's time to poll the sensors again
        lastSatellitePoll = millis();
        
        // Read the temp and humidity, and send two packets of type double whenever the change is sufficient.
        readData();
  
        temp_cal = calibration_T(temp_raw); //Get raw values from BME280
        hum_cal = calibration_H(hum_raw);
        temp_act = (double) temp_cal / 100.0; //Convert raw values to actual.  use round(value*10)/10(.0?) to get 1dp
        hum_act = (double) hum_cal / 1024.0;
  
        printf("Just read temp=%i.%i, hum=%i.%i\n", int(temp_act), int(temp_act * 10) % 10, int(hum_act),
               int(hum_act * 10) % 10);
  
        Transmission latest(deviceID, temp_act, hum_act); //Create new transmission
        
        if (latest.changed(prevPayload, TEMPHYS, HUMHYS)) { //If new values are sufficiently different
          bool delivered = false;
          
          radio.stopListening(); //Pause listening to enable transmitting
  
          while (!delivered) {
  
            printf("Now sending ");
            latest.printCSV(); //A summary of the transmission being sent.
            
            delivered = radio.write(&latest, sizeof(latest)); //Assigns true if transmission is successfully received by base
  
            if (delivered) {
              printf("ok...\n");
            }
            else {
              printf("failed.\n\r");
            }
          }
          prevPayload = latest;
  
        // Continue listening
        radio.startListening();
        }
        // Try again 30s later
        
        //fixme Remove and test
        delay(1000); // Loop poll rate.  Adjust sensor poll rate later to match to reduce power consumption.
      }
    }
  }





  //
  // base role; Device connected to computer which receives transmissions from the satellite
  //

  if (role == role_base) {
   
    if (static_cast<unsigned long int>(millis() - lastDeadmanCheck) > static_cast<unsigned long int>(DEADMANPERIOD)) { // If it's time to check for satellite lifesigns
      //printf("%lu: Checking for life (last checked at %lu)\n", (millis() / 1000), lastDeadmanCheck/1000); // Timestamp (seconds since base start)
      
      checkForLife(liveDevices);
      lastDeadmanCheck = static_cast<unsigned long int>(millis());
      
      for (int i = 1; i <= SATELLITES; i++) { //output summary of life-sign check
        printf("SAT1: %i ", i);
        if (liveDevices[i]) {
          printf("LIVE   ");
        }
        else {
          printf("DEAD   ");
        }
      }
      printf("\n");
    }
    
    Transmission received(-1, 0.0,0.0);

    if (radio.available()) { //If a transmission has been sent from a satellite
      
      radio.read(&received, sizeof(received)); //Read it

      printf("%i;", received.xmitterID); //Output CSV with ID,
      printf("%lu;", (millis() / 1000)); // timestamp (seconds since base start)
      received.printCSV(); //And values
      
      }
      
    // Delay just a little bit to let the other unit
    // make the transition to receiver
    delay(20);
  }
}
