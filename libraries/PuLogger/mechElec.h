#ifndef MECHELEC
#define MECHELEC

#include <Arduino.h>

const int BUZZERPIN = 2;
const int FANPIN = 4;

void startFan();
void stopFan();

#endif