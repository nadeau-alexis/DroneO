#ifndef STEPPER_CONTROLLER_hpp
#define STEPPER_CONTROLLER_hpp
#include <SoftwareSerial.h>


void homing(int* positionPointer_, int STPDir_, int STPStep_, int stepHomeSwitch_, int STPEn_, SoftwareSerial &HC12object, String HC12String_);
void takePosition(int* positionPointer_, int wantedPostion, int STPDir_, int STPStep_, int STPEn_, SoftwareSerial &HC12object, String HC12String_);
void takePurgePosition(int* positionPointer_,int STPDir_, int STPStep_, int STPEn_, SoftwareSerial &HC12object, String HC12String_);
void manualTurning(int STPDir_, int STPStep_, int speed, int btnPin);

#endif
