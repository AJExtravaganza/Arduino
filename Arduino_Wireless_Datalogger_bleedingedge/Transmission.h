#ifndef TRANSMISSION
#define TRANSMISSION

struct Transmission {
  int tempRaw = 0;
  int humRaw = 0;
  Transmission(double temp, double hum) : tempRaw(int(temp * 100)), humRaw(int(hum * 100)) {
    
  }
  float getTemp() { return float(tempRaw) / 100.0;}
  float getHum() { return float(humRaw) / 100.0;}
  bool changed( Transmission other, float tempHys, float humHys) {return (abs(getTemp() - other.getTemp()) > tempHys || abs(getHum() - other.getHum()) > humHys); }
  void printCSV() {printf("%i.%i;%i.%i\n", int(getTemp()), int(getTemp() * 10) % 10, int(getHum()),
              int(getHum() * 10) % 10);}
};

#endif