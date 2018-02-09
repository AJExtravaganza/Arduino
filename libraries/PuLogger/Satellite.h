#ifndef SATELLITE
#define SATELLITE

	//// Alarm thresholds are defined here
const int TEMPSP = 315;
const int HUMSP = 700;
const int TEMPHIGHLIMIT = TEMPSP;
const int TEMPLOWLIMIT = TEMPSP - 25;
const int HUMHIGHLIMIT = HUMSP + 50;
const int HUMLOWLIMIT = HUMSP - 50;
const unsigned long int TEMPALARMGRACEPERIOD = (1000UL *  60UL * 60UL * 5); //5 hours (4.5hrs recovery measured from 24C, which should be far below worst-case in normal use)
const unsigned long int HUMALARMGRACEPERIOD = (1000UL * 60UL * 60UL * 2); //2 hours (1hr recovery measured, with 100% tolerance)


class Satellite { //privatise whatever when you have a final architecture figured out.
private:
	
public:
	Satellite();
	Satellite(int deviceID, bool hasAdditionalSensor, unsigned long int tGrace, int tHigh, int tLow, unsigned long int hGrace, int hHigh, int hLow);
  
	int deviceID;
	bool deviceUp;
	bool hasAdditionalSensor; //should be const, with getter
	unsigned long int lastTransmission;
	
	int tempRawValue[2]; //Raw values in deci-units.  Room for 
  int humRawValue[2];
	
	int tempRawAvg;
	int humRawAvg;
	
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
	
	void update(int sensor, int tempRawValue, int humRawValue, unsigned long int currentTimeElapsed);
	bool tempInRange();
	bool humInRange();
	void procAlarms(unsigned long int currentTimeElapsed);
	void procAlarms(int sensor, unsigned long int currentTimeElapsed);
	void clearAlarms();
	
  float getTemp();
  float getHum();
	float getTemp(int sensor);
  float getHum(int sensor);
	
};

#endif