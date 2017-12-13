#include "Satellite.h"

Satellite::Satellite() : deviceID(-1), deviceUp(false), lastTransmission(0), 
	tempRawValue(-1), humRawValue(-1), 
	tempHighLimit(310), tempLowLimit(240), tempHighAlarm(false), tempLowAlarm(false), 
	tempAlarmGracePeriod(1000UL * 60UL * 60UL),
	humHighLimit(750), humLowLimit(600), humHighAlarm(false), humLowAlarm(false), 
	humAlarmGracePeriod(1000UL * 60UL * 60UL) 
{
	
}

Satellite::Satellite(int deviceID, unsigned long int tGrace, int tHigh, int tLow, unsigned long int hGrace, int hHigh, int hLow) : 
	deviceID(deviceID), deviceUp(false), lastTransmission(0), 
	tempRawValue(-1), humRawValue(-1), 
	tempHighLimit(310), tempLowLimit(240), tempHighAlarm(false), tempLowAlarm(false), 
	tempAlarmGracePeriod(1000UL * 60UL * 60UL),
	humHighLimit(750), humLowLimit(600), humHighAlarm(false), humLowAlarm(false), 
	humAlarmGracePeriod(1000UL * 60UL * 60UL) 
{
	
}

float Satellite::getTemp() {
	return  float(tempRawValue) / 10.0;
}

float Satellite::getHum() {
	return  float(humRawValue) / 10.0;
}