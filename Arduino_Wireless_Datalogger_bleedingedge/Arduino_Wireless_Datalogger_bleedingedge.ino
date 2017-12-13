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
#include "Satellite.h"
#include "BME280.h"


//
// Hardware configuration
//


//When set to false, this seems to initialise every loop, which is concerning
bool deviceWasDown = true; //debug variable for device failure
const unsigned int SATELLITES = 1;
const unsigned int DEVICES = SATELLITES + 1;
bool liveDevices[DEVICES]; //Index corresponds with device ID
unsigned long int satelliteLastTransmissionTime[DEVICES]; //Index corresponds with device ID
unsigned long int lastCheckedIn = 0; // Holds last time satellite contacted base (by successfully sending a transmission)  Used by sats only.

  ////Definable variables that determine transmission frequency////
const unsigned long int CHECKINPERIOD = 1000UL *  10UL;//60UL * 15UL; // Check in once every fifteen minutes
const int DEADMANTHRESHOLD = 10; //Exceeding CHECKINPERIOD by 10% will cause deadman alarm

	//// Hysteresis values in deci-units
const float TEMPHYS = 5;  
const float HUMHYS = 5; 

  ////These are Tingle's magic derived constants.  Do not touch them.////
const unsigned long int DEADMANPERIOD = CHECKINPERIOD * (100UL + static_cast<unsigned long int>(DEADMANTHRESHOLD)) / 100UL;
const unsigned long int SENSORPOLLPERIOD = CHECKINPERIOD / 10UL - 1UL; //Must be <DEADMANTHRESHOLD% of DEADMANPERIOD
const unsigned long int SATELLITELOOPPERIOD = SENSORPOLLPERIOD / 3UL; // Must be <=SENSORPOLLPERIOD

	////Satellite objects for base station
Satellite satellites[DEVICES];

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

/*bool deviceFailure() {
	bool deviceFailed = false;
	
	for (int i = 1; i <= SATELLITES; i++) {

		if (static_cast<unsigned long int>(millis() -satelliteLastTransmissionTime[i]) > DEADMANPERIOD) { 
			liveDevices[i] = false;
			deviceFailed = true;
		}
		else {
			liveDevices[i] = true;
		}
	}
  return deviceFailed;
}
*/

bool deviceFailure(int deviceID) {
  
    if (static_cast<unsigned long int>(millis() - satelliteLastTransmissionTime[deviceID]) > DEADMANPERIOD && millis() > SATELLITELOOPPERIOD) { 
      liveDevices[deviceID] = false;
      //printf("Checking: It's dead.\n");//debug
    }
    else {
      liveDevices[deviceID] = true;
      //printf("Checking: It's not dead.\n");//debug
    }

  	return !liveDevices[deviceID];
}

void setup(void) {

  Serial.begin(57600);

  ////Initialise global arrays
  for (int i = 1; i <= SATELLITES; i++) { //Initialise deadman arrays
    liveDevices[i] = true;
    satelliteLastTransmissionTime[i] = 0UL;
  }
  
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

  Wire.begin();

  writeReg(0xF2, ctrl_hum_reg);
  writeReg(0xF4, ctrl_meas_reg);
  writeReg(0xF5, config_reg);
  readTrim();

  printf_begin();
  //printf("ROLE: %s with ID %i\n\r", role_friendly_name[role], settings.deviceID);

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

  //Commented out for Qt QString compatibility, which can't handle tabs in QString.left()
  //radio.printDetails(); //Outputs detailed information on radio unit and settings
}


void loop(void) {
  
  if (role == role_satellite) {  //Satellite setup
    long long unsigned int lastSensorPoll = 0;
    double temp_act = 0.0, hum_act = 0.0;
    signed long int temp_cal;
    unsigned long int hum_cal;
    Transmission prevPayload(-1, 0.0, 0.0);

    readData();

    temp_cal = calibration_T(temp_raw);
    hum_cal = calibration_H(hum_raw);

    double del_temp_act = 0; //Change in temperature since last transmission
    double del_hum_act = 0; //Change in humidity since last transmission

    printf("CHECKINPERIOD: %lu, DEADMANPERIOD: %lu\n", CHECKINPERIOD, DEADMANPERIOD);

    while (role == role_satellite) { //Satellite main loop
		
      if ((millis() - lastSensorPoll > SENSORPOLLPERIOD) || millis() < 1000UL) { //If it's time to poll the sensors again
        lastSensorPoll = millis();
        
        // Read the temp and humidity, and send two packets of type double whenever the change is sufficient.
        readData();
  
        temp_cal = calibration_T(temp_raw); //Get raw values from BME280
        hum_cal = calibration_H(hum_raw);
        temp_act = (double) temp_cal / 100.0; //Convert raw values to actual.  use round(value*10)/10(.0?) to get 1dp
        hum_act = (double) hum_cal / 1024.0;
  
        printf("Just read temp=%i.%i, hum=%i.%i\n", int(temp_act), int(temp_act * 10) % 10, int(hum_act),
               int(hum_act * 10) % 10);
  
        Transmission latest(deviceID, temp_act, hum_act); //Create new transmission

        printf("Checking %lu > %lu with SATELLITELOOPPERIOD %lu\n", (millis() - lastCheckedIn), CHECKINPERIOD, SATELLITELOOPPERIOD);
        
        if (latest.changed(prevPayload, TEMPHYS, HUMHYS) //If new values are sufficiently different
			      || ((millis() - lastCheckedIn) > CHECKINPERIOD)) { // or it's time to check in with base
          bool delivered = false;
          
          for (int attempt = 1; !delivered && attempt <= 50; ) {
            radio.stopListening(); //Pause listening to enable transmitting
             
            printf("Now sending ");
            latest.printCSV(); //Print a summary of the transmission being sent.
            
            delivered = radio.write(&latest, sizeof(latest)); //Assigns true if transmission is successfully received by base
            if (delivered) {
              printf("Delivered (%i attempts)...\n", attempt);
              prevPayload = latest; //Update record of last transmission successfully sent
			        lastCheckedIn = millis(); //Update record of last check-in with base
            }
            else {
              printf("failed.\n\r");
              delay(13); //Hacky attempt to get out of what may be an ACK/comms-lock
            }

            radio.startListening();
          }
          
				}
			}
          
        //fixme Remove and test
        delay(SATELLITELOOPPERIOD); // Loop poll rate.  Adjust sensor poll rate later to match to reduce power consumption.
    }
  }





  //// Base role; Device connected to computer to receive transmissions from the satellite
  if (role == role_base) {
   
    Transmission received(-1, 0.0,0.0);
			////////SET UP TO USE INDIVIDUAL isUp BOOLS LATER ON////////
    for (int i = 1; i <= SATELLITES; i++) {

      bool wasLive = liveDevices[i];
      //printf("wasLive = %i\n", wasLive);//debug
      
			if (deviceFailure(i)) { //Check for devices that haven't touched base recently.  If such exists,
				//printf("isLive = %i\n", liveDevices[i]);//debug
				if (wasLive) {
				printf("DEVICE %i DOWN at %lu\n", i, (millis() / 1000));
				//liveDevices[i] = false; //redundant
				}
			 else {
				//printf("Device still down.\n");
			 }
			}
			else {
				if (!wasLive) {
					printf("DEVICE %i UP at %lu\n", i, (millis() / 1000));
					//liveDevices[i] = true; //redundant
				}
				else {
					//printf("Device still up.\n");
				}
			}
	  }
  
    if (radio.available()) { //If a satellite transmission is pending
        
      radio.read(&received, sizeof(received)); //Read it
  
      printf("%i;", received.xmitterID); //Output CSV with ID,
      printf("%lu;", (millis() / 1000)); // timestamp (seconds since base start)
      received.printCSV(); //And values
  	  satelliteLastTransmissionTime[received.xmitterID] = millis();
        
    }
        
    // Delay just a little bit to let the other unit
    // make the transition to receiver
    delay(20);
  }
}
