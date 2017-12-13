#ifndef SATELLITE
#define SATELLITE

class Satellite { //privatise whatever when you have a final architecture figured out.
private:
	
public:
	Satellite();
	Satellite(int deviceID, unsigned long int tGrace, int tHigh, int tLow, unsigned long int hGrace, int hHigh, int hLow);
  
	int deviceID;
	bool deviceUp;
	unsigned long int lastTransmission;
	
	int tempRawValue; //Raw values in deci-units
  int humRawValue;
	
	int tempHighLimit; // Temperature alarm properties
	int tempLowLimit;
	bool tempHighAlarm;
	bool tempLowAlarm;
	unsigned long int tempFirstOOR;
	unsigned long int tempAlarmGracePeriod;
	
	int humHighLimit; // Humidity alarm properties
	int humLowLimit;
	bool humHighAlarm;
	bool humLowAlarm;
	unsigned long int humFirstOOR;
	unsigned long int humAlarmGracePeriod;
	
	void update(int tempRawValue, int humRawValue, unsigned long int currentTimeElapsed);
	bool tempInRange();
	bool humInRange();
	void procAlarms(unsigned long int currentTimeElapsed);
	void clearAlarms();
	
  float getTemp();
  float getHum();
  /*void printCSV() {printf("%i;%i;\n", int(getTemp()), int(getHum()),
              int(getHum() * 10) % 10);}*/
};

#endif