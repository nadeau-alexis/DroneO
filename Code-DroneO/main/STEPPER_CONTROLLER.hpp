#ifndef STEPPER_CONTROLLER_hpp
#define STEPPER_CONTROLLER_hpp
#include <SoftwareSerial.h>


void homing(int* positionPointer_, int dirPin_, int stepPin_, int stepHomeSwitch_, int stepperEnable_, SoftwareSerial &HC12object, String HC12String_);
void takePosition(int* positionPointer_, int wantedPostion, int dirPin_, int stepPin_, int stepperEnable_, SoftwareSerial &HC12object, String HC12String_);
void takePurgePosition(int* positionPointer_,int dirPin_, int stepPin_, int stepperEnable_, SoftwareSerial &HC12object, String HC12String_);
void manualTurning(int dirPin_, int stepPin_, int speed, int btnPin);

#endif
