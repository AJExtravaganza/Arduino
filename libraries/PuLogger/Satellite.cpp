#include "Satellite.h"

Satellite::Satellite() : deviceID(-1), deviceUp(true), lastTransmission(0), 
	tempRawValue(-1), humRawValue(-1), 
	tempHighLimit(310), tempLowLimit(230), tempHighAlarm(false), tempLowAlarm(false), 
	tempFirstOOR(0UL), tempAlarmGracePeriod(1000UL * 60UL * 60UL),
	humHighLimit(750), humLowLimit(550), humHighAlarm(false), humLowAlarm(false), 
	humFirstOOR(0UL), humAlarmGracePeriod(1000UL * 60UL * 60UL) 
{
	
}

Satellite::Satellite(int deviceID, unsigned long int tGrace, int tHigh, int tLow, unsigned long int hGrace, int hHigh, int hLow) : 
	deviceID(deviceID), deviceUp(true), lastTransmission(0), 
	tempRawValue(-1), humRawValue(-1), 
	tempHighLimit(310), tempLowLimit(230), tempHighAlarm(false), tempLowAlarm(false), 
	tempFirstOOR(0UL), tempAlarmGracePeriod(1000UL * 60UL * 60UL),
	humHighLimit(750), humLowLimit(550), humHighAlarm(false), humLowAlarm(false), 
	humFirstOOR(0UL), humAlarmGracePeriod(1000UL * 60UL * 60UL) 
{
	
}

void Satellite::update(int tempRawValue, int humRawValue, unsigned long int currentTimeElapsed) {
	Satellite::tempRawValue = tempRawValue;
	Satellite::humRawValue = humRawValue;
	Satellite::lastTransmission = currentTimeElapsed;
	
	Satellite::procAlarms(currentTimeElapsed);
}

bool Satellite::tempInRange() {
	return (tempRawValue > tempLowLimit && tempRawValue < tempHighLimit);
}

bool Satellite::humInRange() {
	return (humRawValue > humLowLimit && humRawValue < humHighLimit);
}

void Satellite::procAlarms(unsigned long int currentTimeElapsed) { // Latching alarms.  Inefficient, but visually elegant.  Change to if-statements later.
	if (tempRawValue < tempLowLimit) {
		if (!tempFirstOOR) {
			tempFirstOOR = currentTimeElapsed;
		}
		else if ((currentTimeElapsed - tempFirstOOR) > tempAlarmGracePeriod) {
			tempLowAlarm = true;
		}
		else {} // Wait out the grace period
	}
	else if (tempInRange()){
		tempFirstOOR = 0UL;
	}
	
	if (tempRawValue > tempHighLimit) {
		if (!tempFirstOOR) {
			tempFirstOOR = currentTimeElapsed;
		}
		else if ((currentTimeElapsed - tempFirstOOR) > tempAlarmGracePeriod) {
			tempHighAlarm = true;
		}
		else {} // Wait out the grace period
	}
	else if (tempInRange()){
		tempFirstOOR = 0UL;
	}

	
	
	if (humRawValue < humLowLimit) { //while(true);
		if (humFirstOOR == 0UL) {
			humFirstOOR = currentTimeElapsed;
		}
		else if ((currentTimeElapsed - humFirstOOR) > humAlarmGracePeriod) {
			humLowAlarm = true;
		}
		else {} // Wait out the grace period
	}
	else if (humInRange()){
		humFirstOOR = 0UL;
	}
	
	if (humRawValue > humHighLimit) {
		if (!humFirstOOR) {
			humFirstOOR = currentTimeElapsed;
		}
		else if ((currentTimeElapsed - humFirstOOR) > humAlarmGracePeriod) {
			humHighAlarm = true;
		}
		else {} // Wait out the grace period
	}
	else if (humInRange()){
		humFirstOOR = 0UL;
	}
}

void Satellite::clearAlarms() {
	tempLowAlarm = false;
	tempHighAlarm = false;
	tempFirstOOR = 0;
	
	humLowAlarm = false;
	humHighAlarm = false;
	humFirstOOR = 0;
}

float Satellite::getTemp() {
	return  float(tempRawValue) / 10.0;
}

float Satellite::getHum() {
	return  float(humRawValue) / 10.0;
}