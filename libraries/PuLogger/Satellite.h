#ifndef SATELLITE
#define SATELLITE

class Satellite {
private:
	int deviceID;
	
public:
	Satellite();
	Satellite(int deviceID);
  
	bool deviceUp;
	
	int tempRawValue; //Raw values in deci-units
  int humRawValue;
	
	bool tempHighAlarm;
	bool tempLowAlarm;
	bool humHighAlarm;
	bool humLowAlarm;
	
  float getTemp();
  float getHum();
  /*void printCSV() {printf("%i;%i;\n", int(getTemp()), int(getHum()),
              int(getHum() * 10) % 10);}*/
};

#endif