#include "PUMP_CONTROLER.hpp"
#include <Arduino.h>

void pumpOn(int pumpPin_)
{
    digitalWrite(pumpPin_, HIGH);
}

void pumpOff(int pumpPin_)
{
    digitalWrite(pumpPin_, LOW);
}

