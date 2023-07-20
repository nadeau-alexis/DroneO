#ifndef hc12_hpp
#define hc12_hpp

#include <SoftwareSerial.h>
#include "string.h"

int checkCommunication(SoftwareSerial &HC12object, String HC12String_);
void returnMessage(SoftwareSerial &HC12object, int returnMessageNumber);

/*
void treuilUnroll(int speed);
void treuilRoll(int speed);
void verinIn(void);
void verinOut(void);
*/
#endif