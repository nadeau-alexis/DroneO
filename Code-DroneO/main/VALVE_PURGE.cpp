#include "VALVE_PURGE.hpp"
#include <Arduino.h>

void valvePurgeOut(int valveOutPin_, int valveInPin_)
{
    digitalWrite(valveOutPin_, HIGH);
	  digitalWrite(valveInPin_, LOW);

   //digitalWrite(relay1, HIGH);
   //digitalWrite(relay2, LOW);
}

void valvePurgeIn(int valveOutPin_, int valveInPin_)
{
    digitalWrite(valveOutPin_, LOW);
	  digitalWrite(valveInPin_, HIGH);
}
