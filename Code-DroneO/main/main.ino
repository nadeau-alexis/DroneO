// ------- Main.ino -------


// ------- EXTERNAL LIBRAIRIES -------
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Encoder.h>
#include <PID_v1.h>

// ------- INTERNAL LIBRAIRIES ------- 
#include "BDC_CONTROLLER.hpp"
#include "STEPPER_CONTROLLER.hpp"
#include "PUMP_CONTROLLER.hpp"
#include "variables.hpp"
#include "HC12.hpp"
#include "string.h"
#include "VALVE_PURGE.hpp"


// ------- CONSTANTS -------
const int phTreuilPin   = 4;  // PH_TREUIL (direction)
const int enTreuilPin   = 5;  // EN_TREUIL (pwm)
const int flowPin       = 7;  // SIG_FL_SENSOR
const int stepperEnable = 8;  // STP_EN
const int stepPin       = 9;  // STP_STEP
const int dirPin        = 10; // STP_DIR
const int pumpPin       = 13; // PWM_POMPE
const int ResetPin      = 15;
const int svBoutPin     = 16; // SV_BOUT
const int svPurgePin    = 17; // SV_PURG
const int hsPlateauPin  = 20; // SIG_LS_PLATEAU
const int hsTreuilPin   = 21; // SIG_LS_TREUIL

const int boutonManuel  = 18;
const int irsensor = 1; // TO DO : remove when irsensor is no longer needed for treuil manual roll and unroll 

// ------- COMMAND NAMES -------
// TO DO : Give each command we want to check for a verbose name 
// and associate it with the number it previously had to add clarity to the code 
// Here is an example, name and associated number might need to be changed
//const int PUMP_COMMAND = 30;


// ------- VARIABLES -------
int position = 0;
int askedCommand = 0;
bool modeNormal = false;
String HC12String;
int D = 0;
int N = 1;

float target_nbTurns = 5; // Variable used to store a target number of winch turns to be used with PID
float pulseByTurn = 2398.31; // Number of pulses by turn of the winch (treuil)
float freq = 0; // Flow meter frequence variable
float flow = 0; // Flow meter flow rate variable

long positionEncTreuil  = -999; // Gives a value to encoder variable

double Setpoint, Input, Output; // PID variables
double Kp=0.5, Ki=0.1, Kd=0; // Put Kd at 0.1 if we want to slow down and arrive smoothly at target

volatile int flowCount = 0; // Volatile variables are better suited for use with interrupts

// ------- OBJECTS -------
SoftwareSerial HC12(11, 12); // HC-12 TX Pin, HC-12 RX Pin
Encoder encTreuil(3, 2); //encA / encB
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


// ------- PROTOTYPES -------

void commandeRemplissageManuel(int wantedBottle, int duree);
void remplissage (int wantedBottle, int duree);
//--------------------------------
void setup() 
{
  Serial.begin(9600);
  HC12.begin(9600);
  
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);
  delay(200); 
  // ---------- pin initialisation ----------
  // --- Motors ---
  // Motor_1 TREUIL control pin initiate
  pinMode(phTreuilPin, OUTPUT);     
  pinMode(enTreuilPin, OUTPUT);    

  // VALVE pin initiate
  pinMode(svBoutPin, OUTPUT);
  pinMode(svPurgePin, OUTPUT);

  // STEPPER BOTTLES control pin initiale
  pinMode(stepPin, OUTPUT); // STEP PIN
  pinMode(dirPin, OUTPUT);  // DIR PIN
  pinMode(stepperEnable, OUTPUT);
  digitalWrite(stepperEnable,HIGH); // Disable stepper motor control
  //pinMode(stepHomeSwitch, INPUT_PULLUP);

  // PUMP control pin initiate
  pinMode(pumpPin, OUTPUT);
 
  // --- LED ---
  pinMode(LED_BUILTIN, OUTPUT);

  // SWITCHES
  pinMode(hsPlateauPin, INPUT); // SIG_LS_PLATEAU
  pinMode(hsTreuilPin, INPUT);  // SIG_LS_TREUIL
  pinMode(boutonManuel, INPUT_PULLUP); 

  // PREPARING MACHINE
  pumpOff(pumpPin);
  valveIn(HC12, HC12String);
  delay(500);
  valvePurgeIn(svPurgePin, svBoutPin);
  delay(4000);

  Serial.println("INITIALISATION");
  digitalWrite(stepperEnable,LOW); //Enable stepper motor control
  delay(1);
  homing(&position, dirPin, stepPin, hsPlateauPin, stepperEnable, HC12, HC12String);
 
}

void loop() 
{
 askedCommand = checkCommunication(HC12, HC12String);
          delay(1);
          if(askedCommand==70)
          {
            Serial.println(askedCommand);
            returnMessage(HC12, 1);
            Resetfct();
          }
          
   else if(stop_loop==false)
  {

          if(askedCommand!=0 && askedCommand<=6)
          {
            Serial.println(askedCommand);
            delay(2000);
            D = checkCommunication(HC12, HC12String);
            commandeRemplissageManuel(askedCommand, D);
            returnMessage(HC12, 1);
            D = 0;
          }
          
        
        else if(askedCommand==8)
          {
            treuilUnroll(1, hsTreuilPin, HC12, HC12String);
            returnMessage(HC12, 1);
          }
        
          else if(askedCommand==9)
          {
            treuilRoll(1, hsTreuilPin, HC12, HC12String);
            Serial.println(stop_loop);
            returnMessage(HC12, 1);
          }
          else if(askedCommand==10)
          {
            returnMessage(HC12, 1);
          }
        
          else if(askedCommand==11)
          {
            modeNormal = true;
            returnMessage(HC12, 1);
          }
        
          else if(askedCommand==12)
          {
            int debut;
            int fin;
            Serial.println(askedCommand);
            N=0;
            debut=millis();
            while(N==0)
            {
               N = checkCommunication(HC12, HC12String);
               delay(10);
            }
            fin=millis();
            Serial.println(N);
            Serial.println(fin-debut);
            treuilUnrollManual(N, 1, hsTreuilPin, irsensor, HC12, HC12String);
            returnMessage(HC12, 1);
          }
        
          else if(askedCommand==13)
          {
            int debut;
            int fin;
            Serial.println(askedCommand);
            N=0;
            debut=millis();
            while(N==0)
            {
               N = checkCommunication(HC12, HC12String);
               delay(10);
            }
            fin=millis();
            Serial.println(N);
            Serial.println(fin-debut);
            treuilRollManual(N, 1, hsTreuilPin, irsensor, HC12, HC12String);
            returnMessage(HC12, 1);
          }
          else if(askedCommand>=21&& askedCommand<=26)
          {
            Serial.println(askedCommand);
            delay(2000);
            D = checkCommunication(HC12, HC12String);
            remplissage(askedCommand-20, D);
            returnMessage(HC12, 1);
            D = 0;
          }
          else if(askedCommand>=51&& askedCommand<=56)
          {
            Serial.println(askedCommand);
            tournerPlateau(askedCommand-50);
            returnMessage(HC12, 1);
          }
        else if(askedCommand==30)
          {
            Serial.println(askedCommand);
            delay(2000);
            D = checkCommunication(HC12, HC12String);
            Serial.println(D);
            pompage(D);
            returnMessage(HC12, 1);
            D = 0;
          }
          else if(askedCommand==40)
          {
            Serial.println(askedCommand);
            tournerPlateauPurge();
            returnMessage(HC12, 1);
          }
          else if(askedCommand==60)
          {
            Serial.println(askedCommand);
            digitalWrite(stepperEnable, LOW); // Enable stepper motor control
            valvePurge();
            returnMessage(HC12, 1);
          }
         else if(askedCommand==70)
          {
            Serial.println(askedCommand);
            returnMessage(HC12, 1);
            Resetfct();
          }
          else if(askedCommand==80)
          {
            Serial.println(askedCommand);
            returnMessage(HC12, 1);
            valvePurgeSansPompage();
          }
         else if(digitalRead(boutonManuel) == 0)
          {
            delay(5);
            if(digitalRead(boutonManuel) == 0) // PROTECTION AU BRUIT
            {
              digitalWrite(stepperEnable, LOW); // Enable stepper motor control
              delay(1);
              manualTurning(14, 15, 100, boutonManuel);
              digitalWrite(stepperEnable, HIGH); // Disable stepper motor control
            }
          }
  }
}

// ----------- FONCTION--------------

void pompage(int duree)
{
  int debut;
  int fin;
  int d=0;
    pumpOn(pumpPin);
    debut1=millis();
    for (int i=0; i<5;i++)
    {
      d1=checkCommunication(HC12, HC12String);
      Serial.println(d);
      if(d==100)
      {
        pumpOff(pumpPin);
        Serial.println("arret d'urgence");
        stop_loop=true;
        return;
      }
      else
      {    
       delay(1000); 
      }
    }
    fin1=millis();
    Serial.println(fin-debut);

}
 
void remplissage (int wantedBottle, int duree)
{
  digitalWrite(stepperEnable, LOW); // Enable stepper motor control
  delay(1);
  //PURGE
  //valvePurge();
  Serial.println(positionPlateau);

  // REMPLISSAGE
  //homing(&position, dirPin, stepPin, hsPlateauPin,stepperEnable, HC12, HC12String);
  if(stop_loop==true){return;}
  delay(50);
  takePosition(&position, wantedBottle, dirPin, stepPin, stepperEnable, HC12, HC12String);
  if(stop_loop==true){return;}
  valveOut(HC12, HC12String);
  if(stop_loop==true){return;}
  pompage(duree);
  if(stop_loop==true){return;}
  valveIn(HC12, HC12String);

  digitalWrite(stepperEnable,HIGH); // Disable stepper motor control
}


void commandeRemplissageManuel(int wantedBottle, int duree)
{
  treuilUnroll(1,hsTreuilPin,HC12, HC12String);
  if(stop_loop==true){return;}
  digitalWrite(stepperEnable,LOW); // Enable stepper motor control
  delay(1);
 // homing(&position, dirPin, stepPin, hsPlateauPin, stepperEnable,  HC12, HC12String);
  if(stop_loop==true){return;}
  delay(50);

  //PURGE

  valvePurge();
  Serial.println(positionPlateau);

  // REMPLISSAGE
  takePosition(&position, wantedBottle, dirPin, stepPin, stepperEnable, HC12, HC12String);
  Serial.println(positionPlateau);

  if(stop_loop==true){return;}
  valveOut(HC12, HC12String);
  if(stop_loop==true){return;}
  pompage(duree);
  if(stop_loop==true){return;}
  valveIn(HC12, HC12String);
  if(stop_loop==true){return;}
  digitalWrite(stepperEnable,HIGH); // Disable stepper motor control
  treuilRoll(1, hsTreuilPin, HC12, HC12String);
}

void tournerPlateau(int wantedBottle)
{
  digitalWrite(stepperEnable,LOW); // Enable stepper motor control
  delay(1);
  //homing(&position, dirPin, stepPin, hsPlateauPin, stepperEnable, HC12, HC12String);
  Serial.println(position);
  if(stop_loop==true){return;}
  delay(50);
  takePosition(&position, wantedBottle, dirPin, stepPin, stepperEnable, HC12, HC12String);
  Serial.println(position);
  Serial.println(positionPlateau);
}


void tournerPlateauPurge()
{
  digitalWrite(stepperEnable, LOW); // Enable stepper motor control
  delay(1);
  Serial.println(position);
  if(stop_loop==true){return;}
  delay(50);
  takePurgePosition(&position, dirPin, stepPin, stepperEnable, HC12, HC12String);
}


void valvePurge()
{
  takePurgePosition(&position, dirPin, stepPin, stepperEnable, HC12, HC12String);
  if(stop_loop==true){return;}
  valvePurgeOut(svPurgePin, svBoutPin);
  delay(4000);
  valveOut(HC12, HC12String);
  if(stop_loop==true){return;}
  pompage(8);
  if(stop_loop==true){return;}
  valveIn(HC12, HC12String);
  if(stop_loop==true){return;}
  delay(500);
  valvePurgeIn(svPurgePin, svBoutPin);
  delay(4000);
}


void valvePurgeSansPompage()
{
  digitalWrite(stepperEnable,LOW); // Enable stepper motor control
  takePurgePosition(&position, dirPin, stepPin, stepperEnable, HC12, HC12String);
  if(stop_loop==true){return;}
  valvePurgeOut(svPurgePin, svBoutPin);
  delay(4000);
  valveOut(HC12, HC12String);
  if(stop_loop==true){return;}
  delay(5000);
  if(stop_loop==true){return;}
  valveIn(HC12, HC12String);
  if(stop_loop==true){return;}
  delay(500);
  valvePurgeIn(svPurgePin, svBoutPin);
  delay(4000);

}


void Resetfct()
{
    digitalWrite(ResetPin, LOW);

}
