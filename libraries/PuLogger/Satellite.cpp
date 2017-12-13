#include "Satellite.h"

Satellite::Satellite() : deviceID(-1), deviceUp(true), lastTransmission(0UL), 
	tempRawValue(-1), humRawValue(-1), 
	tempHighLimit(TEMPHIGHLIMIT), tempLowLimit(TEMPLOWLIMIT), tempHighAlarm(false), tempLowAlarm(false), 
	tempFirstOOR(0UL), tempAlarmGracePeriod(TEMPALARMGRACEPERIOD),
	humHighLimit(HUMHIGHLIMIT), humLowLimit(HUMLOWLIMIT), humHighAlarm(false), humLowAlarm(false), 
	humFirstOOR(0UL), humAlarmGracePeriod(HUMALARMGRACEPERIOD) 
{
	
}

Satellite::Satellite(int deviceID, unsigned long int tGrace, int tHigh, int tLow, unsigned long int hGrace, int hHigh, int hLow) : 
	deviceID(deviceID), deviceUp(true), lastTransmission(0UL), 
	tempRawValue(-1), humRawValue(-1), 
	tempHighLimit(TEMPHIGHLIMIT), tempLowLimit(TEMPLOWLIMIT), tempHighAlarm(false), tempLowAlarm(false), 
	tempFirstOOR(0UL), tempAlarmGracePeriod(TEMPALARMGRACEPERIOD),
	humHighLimit(HUMHIGHLIMIT), humLowLimit(HUMLOWLIMIT), humHighAlarm(false), humLowAlarm(false), 
	humFirstOOR(0UL), humAlarmGracePeriod(HUMALARMGRACEPERIOD) 
{
	
}

	//// Update satellite snapshot from relevant fields of a transmission.
void Satellite::update(int tempRawValue, int humRawValue, unsigned long int currentTimeElapsed) {
	Satellite::tempRawValue = tempRawValue;
	Satellite::humRawValue = humRawValue;
	Satellite::lastTransmission = currentTimeElapsed;
	
	Satellite::procAlarms(currentTimeElapsed);
}

	//// Is temperature between the high and low limits?
bool Satellite::tempInRange() {
	return (tempRawValue > tempLowLimit && tempRawValue < tempHighLimit);
}

	//// Is humidity between the high and low limits?
bool Satellite::humInRange() {
	return (humRawValue > humLowLimit && humRawValue < humHighLimit);
}

  //// Check alarm triggers and update alarm status
void Satellite::procAlarms(unsigned long int currentTimeElapsed) {
	
	if (tempRawValue < tempLowLimit) { // If value is out of bounds
		if (!tempFirstOOR) { // If it only just went OOB
			tempFirstOOR = currentTimeElapsed; // Record grace-period start-time.
		}
		else if ((currentTimeElapsed - tempFirstOOR) > tempAlarmGracePeriod) { // Otherwise, check if OOB state duration exceeds grace period.
			tempLowAlarm = true; //If so, set alarm state.
		}
		else {} // If not, wait out the grace period
	}
	else if (tempInRange()){ // If the value is within bounds
		tempFirstOOR = 0UL; // Reset grace period start time to inactive state
	}
	
	if (tempRawValue > tempHighLimit) { // As above
		if (!tempFirstOOR) {
			tempFirstOOR = currentTimeElapsed;
		}
		else if ((currentTimeElapsed - tempFirstOOR) > tempAlarmGracePeriod) {
			tempHighAlarm = true;
		}
		else {} 
	}
	else if (tempInRange()){
		tempFirstOOR = 0UL;
	}

	
	
	if (humRawValue < humLowLimit) { // As above
		if (humFirstOOR == 0UL) {
			humFirstOOR = currentTimeElapsed;
		}
		else if ((currentTimeElapsed - humFirstOOR) > humAlarmGracePeriod) {
			humLowAlarm = true;
		}
		else {} 
	}
	else if (humInRange()){
		humFirstOOR = 0UL;
	}
	
	if (humRawValue > humHighLimit) { // As above
		if (!humFirstOOR) {
			humFirstOOR = currentTimeElapsed;
		}
		else if ((currentTimeElapsed - humFirstOOR) > humAlarmGracePeriod) {
			humHighAlarm = true;
		}
		else {}
	}
	else if (humInRange()){
		humFirstOOR = 0UL;
	}
}

	////Reset alarm state and grace period tracking
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