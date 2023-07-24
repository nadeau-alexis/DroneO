#include "STEPPER_CONTROLLER.hpp"
#include "HC12.hpp"
#include "variables.hpp"
#include <SoftwareSerial.h>
#include <Arduino.h>


// FONCTION DE HOMING -> CYCLE DE ZERO MACHINE
void homing(int* positionPointer_, int dirPin_, int stepPin_, int stepHomeSwitch_, int stepperEnable_, SoftwareSerial &HC12object, String HC12String_)
{
    digitalWrite(dirPin_, HIGH);
    long j = 0;
    int vitesse = 0;

    while(digitalRead(stepHomeSwitch_)==HIGH)
    { 
      if(checkCommunication(HC12object, HC12String_)!=100)
       {
        if(j<300)
        {
            vitesse = 350 - j;
            j++;
        }
        delayMicroseconds(vitesse);
        digitalWrite(stepPin_, HIGH);
        delayMicroseconds(vitesse);
        digitalWrite(stepPin_, LOW);
       }
       else
       {
            digitalWrite(stepperEnable_, HIGH);
            Serial.println("arret d'urgence");
            stop_loop=true;
            return;
        }
    }

    digitalWrite(dirPin_, LOW);
    delay(50);

    for(int k=0; k<1600; k++)
    {
      if(checkCommunication(HC12object, HC12String_)!=100)
       {
      
            if(k<200)
            {
                vitesse = 350 - j;
            }
                delayMicroseconds(vitesse);
                digitalWrite(stepPin_, HIGH);
                delayMicroseconds(vitesse);
                digitalWrite(stepPin_, LOW);
        }
        else
       {
            digitalWrite(stepperEnable_, HIGH);
            stop_loop=true;
            Serial.println("arret d'urgence");
            return;
        }
    }

    digitalWrite(dirPin_, HIGH);
    delay(50);

    while(digitalRead(stepHomeSwitch_)==HIGH)
    {
      if(checkCommunication(HC12object, HC12String_)!=100)
       {
            if(j<200)
            {
                vitesse = 450 - j;
                j++;
            }
            delayMicroseconds(vitesse);
            digitalWrite(stepPin_, HIGH);
            delayMicroseconds(vitesse);
            digitalWrite(stepPin_, LOW);
       }
       else
       {
            digitalWrite(stepperEnable_,HIGH);
            stop_loop=true;
            Serial.println("arret d'urgence");
            return;
        }
    }

    digitalWrite(dirPin_, LOW);
    delay(50);

    for(int i=0; i<4900; i++)
    {
      if(checkCommunication(HC12object, HC12String_)!=100)
       {
            if(i<700)
            {
                vitesse = 750 - i;
            }
            else if(i>(4900-700))
            {
                vitesse = 50 + (750-(4900-i));
            }
            else
            {
                vitesse = 50;
            }
    
            delayMicroseconds(vitesse);
            digitalWrite(stepPin_, HIGH);
            delayMicroseconds(vitesse);
            digitalWrite(stepPin_, LOW);
        }
        else
        {
            digitalWrite(stepperEnable_,HIGH);
            stop_loop=true;
            Serial.println("arret d'urgence");
            return; 
         }
    }
    *positionPointer_ = 1;
}

// FONCTION QUI PRENDS LA POSITION DEMANDE
void takePosition(int* positionPointer_, int wantedPostion, int dirPin_, int stepPin_, int stepperEnable_, SoftwareSerial &HC12object, String HC12String_)
{
    long map[]={0,2600,11200,19400,27500,35650,43700,0,0,0,0,6900,15300,23450,31575,39675,47725};  
    //long map[]={0,2600,9050,15300,27500,35650,43700,31575};
    
    long difference = map[wantedPostion] - map[*positionPointer_];

    int vitesse = 0;

    if(difference<0)
    {
        digitalWrite(dirPin_, HIGH);
        difference = abs(difference);
    }
    else
    {
        digitalWrite(dirPin_, LOW);
    }

    for(long i=0; i<difference; i++)
    { 
       if(checkCommunication(HC12object, HC12String_)!=100)
       { 
          if(i<700)
          {
              vitesse = 750 - i;
          }
          else if(i>(difference-700))
          {
              vitesse = 50 + (750-(difference-i));
          }
          else
          {
              vitesse = 50;
          }

          delayMicroseconds(vitesse);
          digitalWrite(stepPin_, HIGH);
          delayMicroseconds(vitesse);
          digitalWrite(stepPin_, LOW);
        }
        else
        {
            digitalWrite(stepperEnable_,HIGH);
            Serial.println("arret d'urgence");
            stop_loop=true;
            return;
        }
    }
     
    *positionPointer_ = wantedPostion;
     positionPlateau=wantedPostion;
}
//FONCTION QUI PERMET AU CARROUSSEL DE SE POSITIONNER ENTRE DEUX BOUTEILLES POUR EFFECTUER LA PURGE

void takePurgePosition(int* positionPointer_, int dirPin_, int stepPin_, int stepperEnable_, SoftwareSerial &HC12object, String HC12String_)
{
    positionPlateau= *positionPointer_;
    long map[]={0,2600,11200,19400,27500,35650,43700,0,0,0,0,6900,15300,23450,31575,39675,47725};
    long difference = map[positionPlateau + 10] - map[positionPlateau];

    int vitesse = 0;

    if(difference<0)
    {
        digitalWrite(dirPin_, HIGH);
        difference = abs(difference);
    }
    else
    {
        digitalWrite(dirPin_, LOW);
    }

    for(long i=0; i<difference; i++)
    { 
       if(checkCommunication(HC12object, HC12String_)!=100)
       { 
          if(i<700)
          {
              vitesse = 750 - i;
          }
          else if(i>(difference-700))
          {
              vitesse = 50 + (750-(difference-i));
          }
          else
          {
              vitesse = 50;
          }
          delayMicroseconds(vitesse);
          digitalWrite(stepPin_, HIGH);
          delayMicroseconds(vitesse);
          digitalWrite(stepPin_, LOW);
        }
        else
        {
            digitalWrite(stepperEnable_,HIGH);
            Serial.println("arret d'urgence");
            stop_loop=true;
            return;
        }
    }
    *positionPointer_=positionPlateau+10;
}

//FONCTION QUI PERMET AU CARROUSSEL DE TOURNER MANUELLEMENT
void manualTurning(int dirPin_, int stepPin_, int speed, int btnPin)
{
    digitalWrite(dirPin_, LOW);
    while(digitalRead(btnPin) == 0)
    {
        delayMicroseconds(speed);
        digitalWrite(stepPin_, HIGH);
        delayMicroseconds(speed);
        digitalWrite(stepPin_, LOW);
    }
 }
