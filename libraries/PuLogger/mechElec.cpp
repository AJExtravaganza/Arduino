#include "mechElec.h"

void startFan() {
	digitalWrite(FANPIN, 1);
}

void stopFan() {
	digitalWrite(FANPIN, 0);
}
