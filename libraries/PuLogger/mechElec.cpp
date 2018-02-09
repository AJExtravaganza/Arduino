#include "mechElec.h"

void startFan() {
	digitalWrite(FANPIN, 1);
}

void stopFan() {
	digitalWrite(FANPIN, 0);
}

void startHeat() {
	digitalWrite(HEATPIN, 1);
}

void stopHeat() {
	digitalWrite(HEATPIN, 0);
}