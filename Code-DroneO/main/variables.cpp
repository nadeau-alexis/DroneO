#include <Arduino.h>
#include "variables.hpp"

bool stop_loop = false;  // Define the global varible with an initial value.
int positionPlateau = 0;
long positionEncTreuil = -999;
double Setpoint, Input, Output; // PID variables
const int PHTreuil = 4;
const int ENTreuil = 5;
const float pulseByTurn = 2398.31; // Number of pulses by turn of the winch (treuil)

void printGlobal() {
  Serial.println(stop_loop);
  Serial.println(positionPlateau);
}
