#ifndef TRANSMISSION
#define TRANSMISSION

struct Transmission {
  int xmitterID = -1;
  int tempRaw[2] = {0, 0}; // Make private, since it's in hundredth-units
  int humRaw[2] = {0, 0}; // And this also
  Transmission(int xmitterID, double temp, double hum) : xmitterID(xmitterID), tempRaw({int(temp * 100), 0}), humRaw({int(hum * 100), 0}) { //fixme need to provide both values in constructor, or neither.
    
  }
	int getRawTemp(int sensor) {return (tempRaw[sensor] + 5) / 10;}
	int getRawHum(int sensor) {return (humRaw[sensor] + 5) / 10;}
  float getTemp(int sensor) { return float(tempRaw[sensor]) / 10.0;}
  float getHum(itn sensor) { return float(humRaw[sensor]) / 10.0;}
  bool changed( Transmission other, float tempHys, float humHys) {return (abs(getTemp(0) - other.getTemp(0)) > tempHys || abs(getHum(0) - other.getHum(0)) > humHys) || getTemp(1) - other.getTemp(1)) > tempHys || abs(getHum(1) - other.getHum(1)) > humHys; } //change to raw values later?
  void printCSV() {printf("%i;%i;\n", int(getTemp()), int(getHum()),
              int(getHum() * 10) % 10);}
};

#endif