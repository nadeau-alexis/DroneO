#ifndef VARIABLES_HPP
#define VARIABLES_HPP

extern bool stop_loop;
extern int positionPlateau;
extern long positionEncTreuil;
extern double Setpoint, Input, Output; // PID variables
extern const int PHTreuil, ENTreuil;
extern const float pulseByTurn;

void printGlobal();
#endif
