//// TO DO
//// Work on a low-power mode such that radio is only active when necessary
//// Work on implementing controls signal communication so that base station can drive outputs on the satellite
//// Sanity check for erroneous values (temp/hum should not change more than 1-2deg at a time, and value in array should be limited to this range of change
//// Remember to comment out debug for final deployment
//// Why is it sending non-changed packets?  Investigate.

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#include <dht.h>

//
// Hardware configuration
//

dht DHT;
const double TEMPHYSTERESIS = 0.5;  // Change in temp or humi must exceed this magnitude to trigger an update
const double HUMHYSTERESIS = 0.5; // Calibration factor applied to temp measurement

////BME280 && SPI stuff
#include <Wire.h>

#define BME280_ADDRESS 0x76
unsigned long int hum_raw, temp_raw, pres_raw;
signed long int t_fine;

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
int8_t dig_H1;
int16_t dig_H2;
int8_t dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t dig_H6;

////BME280-specific functions////

void readTrim() {
  uint8_t data[32], i = 0;
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 24);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }

  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xA1);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 1);
  data[i] = Wire.read();
  i++;

  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xE1);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 7);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];
  dig_H1 = data[24];
  dig_H2 = (data[26] << 8) | data[25];
  dig_H3 = data[27];
  dig_H4 = (data[28] << 4) | (0x0F & data[29]);
  dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
  dig_H6 = data[31];
}

void writeReg(uint8_t reg_address, uint8_t data) {
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(reg_address);
  Wire.write(data);
  Wire.endTransmission();
}


void readData() {
  int i = 0;
  uint32_t data[8];
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 8);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
  temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
  hum_raw = (data[6] << 8) | data[7];
}


signed long int calibration_T(signed long int adc_T) {

  signed long int var1, var2, T;
  var1 = ((((adc_T >> 3) - ((signed long int) dig_T1 << 1))) * ((signed long int) dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int) dig_T1)) * ((adc_T >> 4) - ((signed long int) dig_T1))) >> 12) *
          ((signed long int) dig_T3)) >> 14;

  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

unsigned long int calibration_P(signed long int adc_P) {
  signed long int var1, var2;
  unsigned long int P;
  var1 = (((signed long int) t_fine) >> 1) - (signed long int) 64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int) dig_P6);
  var2 = var2 + ((var1 * ((signed long int) dig_P5)) << 1);
  var2 = (var2 >> 2) + (((signed long int) dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int) dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((signed long int) dig_P1)) >> 15);
  if (var1 == 0) {
    return 0;
  }
  P = (((unsigned long int) (((signed long int) 1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (P < 0x80000000) {
    P = (P << 1) / ((unsigned long int) var1);
  }
  else {
    P = (P / (unsigned long int) var1) * 2;
  }
  var1 = (((signed long int) dig_P9) * ((signed long int) (((P >> 3) * (P >> 3)) >> 13))) >> 12;
  var2 = (((signed long int) (P >> 2)) * ((signed long int) dig_P8)) >> 13;
  P = (unsigned long int) ((signed long int) P + ((var1 + var2 + dig_P7) >> 4));
  return P;
}

unsigned long int calibration_H(signed long int adc_H) {
  signed long int v_x1;

  v_x1 = (t_fine - ((signed long int) 76800));
  v_x1 = (((((adc_H << 14) - (((signed long int) dig_H4) << 20) - (((signed long int) dig_H5) * v_x1)) +
            ((signed long int) 16384)) >> 15) * (((((((v_x1 * ((signed long int) dig_H6)) >> 10) *
                                                     (((v_x1 * ((signed long int) dig_H3)) >> 11) +
                                                      ((signed long int) 32768))) >> 10) +
                                                   ((signed long int) 2097152)) *
                                                  ((signed long int) dig_H2) + 8192) >> 14));
  v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int) dig_H1)) >> 4));
  v_x1 = (v_x1 < 0 ? 0 : v_x1);
  v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
  return (unsigned long int) (v_x1 >> 12);
}

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 
RF24 radio(9, 10);
bool tempDelivered = false; //Only used for satellites.  Declared here for persistence over different loop() iterations
bool humDelivered = false;


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

// The role of the current running sketch
role_e role = role_base; //THIS IS NOT THE ONE TO CHANGE


void setup(void) {
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
  readTrim();                    //

  //
  // Print preamble
  //

  if (true) { // Change to detect BME280 on SPI bus
    role = role_base;//////////////////////////////////////////////////////////////////////////////////////////////////////
  }

  printf_begin();
  printf("ROLE: %s\n\r", role_friendly_name[role]);

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
  
  //
  // xmitter role.  Update sensor values, then send them together as an int of format TTHH if they have changed
  //
  
  if (role == role_satellite) {
    double temp_act = 0.0, hum_act = 0.0;
    signed long int temp_cal;
    unsigned long int hum_cal;

    readData();

    temp_cal = calibration_T(temp_raw);
    hum_cal = calibration_H(hum_raw);

    double initTemp = (double) temp_cal / 100.0;
    double initHum = (double) hum_cal / 1024.0;


    double prev_temp_act = 0;
    double prev_hum_act = 0;

    double del_temp_act = 0;
    double del_hum_act = 0;

    printf("Init with T=%i.%i, H=%i.%i\n", int(initTemp), int(initTemp * 10) % 10, int(initHum),
           int(initHum * 10) % 10);

    while (role == role_satellite) {

      // Read the temp and humidity, and send two packets of type double whenever the change is sufficient.
      readData();

      temp_cal = calibration_T(temp_raw); //Get raw values from BME280
      hum_cal = calibration_H(hum_raw);

      temp_act = (double) temp_cal / 100.0; //Convert raw values to actual.  use round(value*10)/10(.0?) to get 1dp
      hum_act = (double) hum_cal / 1024.0;

      printf("Just read temp=%i.%i, hum=%i.%i\n", int(temp_act), int(temp_act * 10) % 10, int(hum_act),
             int(hum_act * 10) % 10);

      del_temp_act = temp_act - prev_temp_act;
      del_hum_act = hum_act - prev_hum_act;

      /*
     printf("HYS*100: %i\n", (int(TEMPHYSTERESIS*100)));
     
     printf("temp*100: %i\n", int(temp_act* 100));
     printf("prev_temp*100: %i\n", int(prev_temp_act* 100));
     printf("del_temp*100: %i\n", int(del_temp_act* 100));
     
     printf("hum*100: %i\n", int(hum_act* 100));
     printf("prev_hum*100: %i\n", int(prev_hum_act* 100));
     printf("del_hum*100: %i\n\n", int(del_hum_act* 100));
     */
      // Only transmit if the temp or humi has changed sufficiently since last xmit
      if (abs(del_temp_act) > TEMPHYSTERESIS || abs(del_hum_act) > HUMHYSTERESIS) {

        tempDelivered = false; //There is a new temperature value to send
        humDelivered = false;

        while (!tempDelivered) {


          // Stop listening so we can talk.
          radio.stopListening();

          printf("Now sending temp %i.%i...", int(temp_act), int(temp_act * 10) % 10);
          tempDelivered = radio.write(&temp_act, sizeof(double));

          if (tempDelivered) {
            printf("ok...\n");
          }
          else {
            printf("failed.\n\r");
          }
        }
        prev_temp_act = temp_act;

        radio.write(0,0); // This can fix a failed subsequent xmit if radio.available isn't called on read.

        while (!humDelivered) {
          printf("Now sending hum %i.%i...", int(hum_act), int(hum_act * 10) % 10);
          humDelivered = radio.write(&hum_act, sizeof(double));

          if (humDelivered) {
            printf("ok...\n\n");
          }
          else {
            printf("failed.\n\r");
          }
        }
        prev_hum_act = hum_act;

      }


      // Continue listening
      radio.startListening();

      // Try again 30s later
      delay(1000); // Loop poll rate.  Adjust sensor poll rate later to match to reduce power consumption.
    }
  }





  //
  // base role; Device connected to computer which receives transmissions from the satellite
  //

  if (role == role_base) {
    // if there is data ready
    if (radio.available()) {
      double temp = 0.0;
      double hum = 0.0;
      bool tempReceived = false;
      bool humReceived = false;
      while (!tempReceived) {
        // Fetch the payload, and see if this was the last one.
        tempReceived = radio.read(&temp, sizeof(double));


      }
       //printf("temp=%i.%i", int(temp), int(temp * 10) % 10);
       
      while (!humReceived) {
        if (radio.available()) {
          humReceived = radio.read(&hum, sizeof(double));
          //printf("hum=%i.%i\n", int(hum), int(hum * 10) % 10);
        }
      }
     
      double t = temp ; //Data ends up in the wrong car for no goddamn reason I can think of.  Code in xmit IDs for each variable later.
      temp = hum;
      hum = t;
      
      printf("Just read temp=%i.%i, hum=%i.%i\n", int(temp), int(temp * 10) % 10, int(hum), int(hum * 10) % 10);
      
    }
    // Delay just a little bit to let the other unit
    // make the transition to receiver
    delay(20);
  }
}
