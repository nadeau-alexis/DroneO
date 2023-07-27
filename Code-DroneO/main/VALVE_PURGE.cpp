#include "VALVE_PURGE.hpp"
#include <Arduino.h>

void valvePurgeOut(int SVPurg_, int SVBout_)
{
    digitalWrite(SVPurg_, HIGH);
	  digitalWrite(SVBout_, LOW);

   //digitalWrite(relay1, HIGH);
   //digitalWrite(relay2, LOW);
}

void valvePurgeIn(int SVPurg_, int SVBout_)
{
    digitalWrite(SVPurg_, LOW);
	  digitalWrite(SVBout_, HIGH);
}
