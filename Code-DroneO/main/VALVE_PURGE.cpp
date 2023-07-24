#include "VALVE_PURGE.hpp"
#include <Arduino.h>

void valvePurgeOut(int svPurgePin_, int svBoutPin_)
{
    digitalWrite(svPurgePin_, HIGH);
	  digitalWrite(svBoutPin_, LOW);

   //digitalWrite(relay1, HIGH);
   //digitalWrite(relay2, LOW);
}

void valvePurgeIn(int svPurgePin_, int svBoutPin_)
{
    digitalWrite(svPurgePin_, LOW);
	  digitalWrite(svBoutPin_, HIGH);
}
