#include <Arduino.h>
#include "variables.hpp"

bool stop_loop = false;  // Define the global varible with an initial value.
int positionPlateau = 0; 
void printGlobal() {
  Serial.println(stop_loop);
  Serial.println(positionPlateau);
}
