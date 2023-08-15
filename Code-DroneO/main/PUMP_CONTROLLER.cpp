#include "PUMP_CONTROLLER.hpp"
#include <Arduino.h>

void pumpOn(int PWMPompe_)
{
    digitalWrite(PWMPompe_, HIGH);
}

void pumpOff(int PWMPompe_)
{
    digitalWrite(PWMPompe_, LOW);
}