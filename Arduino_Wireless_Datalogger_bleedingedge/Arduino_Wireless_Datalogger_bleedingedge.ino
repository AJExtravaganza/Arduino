//// TO DO
//// Work on a low-power mode such that radio is only active when necessary
//// Work on implementing controls signal communication so that base station can drive outputs on the satellite
//// Sanity check for erroneous values (temp/hum should not change more than 1-2deg at a time, and value in array should be limited to this range of change
//// Remember to comment out debug for final deployment
//// Why is it sending non-changed packets?  Investigate.


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

const float TEMPHYS = 0.5;  // Hysteresis values
const float HUMHYS = 0.5; 
bool tempDelivered = false; //Only used for satellites.  Declared here for persistence over different loop() iterations
bool humDelivered = false;  // Move these to one of the infinite internal loops at a later date

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 
RF24 radio(9, 10);


//
// Topology
//

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL};


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


// Declare role
role_e role = role_base;


void setup(void) {
  ////Load settings from EEPROM and assign role
  DeviceSettings settings;
  settings.read();
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

  //
  // Print preamble
  //

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

  //
  // Open pipes to other nodes for communication
  //

  // This simple sketch opens two pipes for these two nodes to communicate
  // back and forth.
  // Open 'our' pipe for writing
  // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)

  if (role == role_base) {
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1, pipes[1]);
  }
  else {
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1, pipes[0]);
  }

  //
  // Start listening
  //

  radio.startListening();

  //
  // Dump the configuration of the rf unit for debugging
  //

  radio.printDetails();
}


void loop(void) {
  
  if (role == role_satellite) {
    double temp_act = 0.0, hum_act = 0.0;
    signed long int temp_cal;
    unsigned long int hum_cal;
    Transmission prevPayload(0.0,0.0);

    readData();

    temp_cal = calibration_T(temp_raw);
    hum_cal = calibration_H(hum_raw);

    //double initTemp = (double) temp_cal / 100.0;
    //double initHum = (double) hum_cal / 1024.0;


    //double prev_temp_act = 0;
    //double prev_hum_act = 0;

    double del_temp_act = 0;
    double del_hum_act = 0;

    //printf("Init with T=%i.%i, H=%i.%i\n", int(initTemp), int(initTemp * 10) % 10, int(initHum),
   //        int(initHum * 10) % 10);

    while (role == role_satellite) {

      // Read the temp and humidity, and send two packets of type double whenever the change is sufficient.
      readData();

      temp_cal = calibration_T(temp_raw); //Get raw values from BME280
      hum_cal = calibration_H(hum_raw);
      temp_act = (double) temp_cal / 100.0; //Convert raw values to actual.  use round(value*10)/10(.0?) to get 1dp
      hum_act = (double) hum_cal / 1024.0;

      printf("Just read temp=%i.%i, hum=%i.%i\n", int(temp_act), int(temp_act * 10) % 10, int(hum_act),
             int(hum_act * 10) % 10);

      Transmission latest(temp_act, hum_act);
      
      if (latest.changed(prevPayload, TEMPHYS, HUMHYS)) {
        bool delivered = false;
        
        radio.stopListening();

        while (!delivered) {

          printf("Now sending ");
          latest.printCSV();
          
          delivered = radio.write(&latest, sizeof(latest));

          if (delivered) {
            printf("ok...\n");
          }
          else {
            printf("failed.\n\r");
          }
        }
        prevPayload = latest;

        //radio.write(0,0); // This can fix a failed subsequent xmit if radio.available isn't called on read.

      // Continue listening
      radio.startListening();
      }
      // Try again 30s later
      delay(1000); // Loop poll rate.  Adjust sensor poll rate later to match to reduce power consumption.
    }
  }





  //
  // base role; Device connected to computer which receives transmissions from the satellite
  //

  if (role == role_base) {
    
    Transmission received(0.0,0.0);

    if (radio.available()) {
      
      radio.read(&received, sizeof(received));

      printf("%lu;", (millis() / 1000)); // Timestamp (seconds since base start)
      received.printCSV();
      
      }
      
    // Delay just a little bit to let the other unit
    // make the transition to receiver
    delay(20);
  }
}
