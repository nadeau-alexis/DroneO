#include "PUMP_CONTROLER.hpp"
#include <Arduino.h>

void pump1on(int pump1Pin_)
{
    digitalWrite(pump1Pin_, HIGH);
}

void pump1off(int pump1Pin_)
{
    digitalWrite(pump1Pin_, LOW);
}

void pump2on(int pump2Pin_)
{
    digitalWrite(pump2Pin_, HIGH);
}

void pump2off(int pump2Pin_)
{
    digitalWrite(pump2Pin_, LOW);
}
