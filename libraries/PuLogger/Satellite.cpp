#include "Satellite.h"

Satellite::Satellite() : deviceID(-1), deviceUp(false), tempRawValue(-1), humRawValue(-1), tempHighAlarm(false), tempLowAlarm(false), humHighAlarm(false), humLowAlarm(false) {
	
}

Satellite::Satellite(int deviceID) : deviceID(deviceID), deviceUp(false), tempRawValue(-1), humRawValue(-1), tempHighAlarm(false), tempLowAlarm(false), humHighAlarm(false), humLowAlarm(false) {
	
}

float Satellite::getTemp() {
	return  float(tempRawValue) / 10.0;
}

float Satellite::getHum() {
	return  float(humRawValue) / 10.0;
}