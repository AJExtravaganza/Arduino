#ifndef TRANSMISSION
#define TRANSMISSION

struct Transmission {
  int xmitterID = -1;
  int tempRaw[2] = {0, 0}; // Make private, since it's in hundredth-units
  int humRaw[2] = {0, 0}; // And this also
	
	//Constructor for single-sensor satellites
  Transmission(int xmitterID, double temp, double hum) : xmitterID(xmitterID), tempRaw({int(temp * 100), -1}), humRaw({int(hum * 100), -1}) {
		
  }
	
	// Constructor for double-sensor satellites
	Transmission(int xmitterID, double temp_0, double hum_0, double temp_1, double hum_1) : xmitterID(xmitterID), tempRaw({int(temp_0 * 100), int(temp_1 * 100)}), humRaw({int(hum_0 * 100), int(hum_1 * 100)})
	{
		
  }
	
	int getRawTemp(int sensor) {return (tempRaw[sensor] + 5) / 10;}
	int getRawHum(int sensor) {return (humRaw[sensor] + 5) / 10;}
  float getTemp(int sensor) { return float(tempRaw[sensor]) / 10.0;}
  float getHum(int sensor) { return float(humRaw[sensor]) / 10.0;}
  bool changed( Transmission other, float tempHys, float humHys) {return (abs(getTemp(0) - other.getTemp(0)) > tempHys || abs(getHum(0) - other.getHum(0)) > humHys) || abs(getTemp(1) - other.getTemp(1)) > tempHys || abs(getHum(1) - other.getHum(1)) > humHys; } //change to raw values later?
  void printCSV() {
		printf("%i;%i;%i;%i;\n", int(getRawTemp(0)), int(getRawHum(0)), int(getRawTemp(1)), int(getRawHum(1)));
	}
};

#endif