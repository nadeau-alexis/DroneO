#ifndef BDC_CONTROLLER_hpp
#define BDC_CONTROLLER_hpp
#include <SoftwareSerial.h>


void treuilUnroll(int speed, int limitswitch, SoftwareSerial &HC12object, String HC12String_);
void treuilRoll(int speed, int limitswitch, SoftwareSerial &HC12object, String HC12String_);
void treuilRollManual(int n, int speed, int limitswitch, int irsensor, SoftwareSerial &HC12object, String HC12String_);
void treuilUnrollManual(int n, int speed, int limitswitch,int irsensor, SoftwareSerial &HC12object, String HC12String_);
void valveIn(SoftwareSerial &HC12object, String HC12String_);
void valveOut(SoftwareSerial &HC12object, String HC12String_);

#endif
