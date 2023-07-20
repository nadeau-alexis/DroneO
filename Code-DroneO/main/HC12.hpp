#ifndef hc12_hpp
#define hc12_hpp

#include <SoftwareSerial.h>
#include "string.h"

int checkCommunication(SoftwareSerial &HC12object, String HC12String_);
void returnMessage(SoftwareSerial &HC12object, int returnMessageNumber);

#endif