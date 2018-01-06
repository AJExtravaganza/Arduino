#ifndef TRANSMISSION
#define TRANSMISSION

struct Transmission {
  int xmitterID = -1;
  int tempRaw[2] = {0, 0}; // Make private, since it's in hundredth-units
  int humRaw[2] = {0, 0}; // And this also
	
	//Constructor for single-sensor satellites
  Transmission(int xmitterID, int temp, int hum) : xmitterID(xmitterID), tempRaw{temp, -1}, humRaw{hum, -1} {
		
  }
	
	// Constructor for double-sensor satellites
	Transmission(int xmitterID, int temp_0, int hum_0, int temp_1, int hum_1) : xmitterID(xmitterID), tempRaw{temp_0, temp_1}, humRaw{hum_0, hum_1}
	{
		
  }
	
	int getRawTemp(int sensor) {return tempRaw[sensor];}
	int getRawHum(int sensor) {return humRaw[sensor];}
  //float getTemp(int sensor) { return float(tempRaw[sensor]) / 10.0;}
  //float getHum(int sensor) { return float(humRaw[sensor]) / 10.0;}
  bool changed( Transmission other, float tempHys, float humHys) {return (abs(getRawTemp(0) - other.getRawTemp(0)) > tempHys || abs(getRawHum(0) - other.getRawHum(0)) > humHys) || abs(getRawTemp(1) - other.getRawTemp(1)) > tempHys || abs(getRawHum(1) - other.getRawHum(1)) > humHys; } //change to raw values later?
  void printCSV() {
		printf("%i;%i;%i;%i;\n", int(getRawTemp(0)), int(getRawHum(0)), int(getRawTemp(1)), int(getRawHum(1)));
	}
};

#endif