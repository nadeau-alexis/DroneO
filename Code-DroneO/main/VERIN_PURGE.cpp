#include "VERIN_PURGE.hpp"
#include <Arduino.h>

void verinpurgeOut(int verinOutPin_, int verinInPin_)
{
    digitalWrite(verinOutPin_, HIGH);
	  digitalWrite(verinInPin_, LOW);

   //digitalWrite(relay1, HIGH);
   //digitalWrite(relay2, LOW);
}

void verinpurgeIn(int verinOutPin_, int verinInPin_)
{
    digitalWrite(verinOutPin_, LOW);
	  digitalWrite(verinInPin_, HIGH);
}
