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

const int ENCFBB        = 2;  // FB_B
const int ENCFBA        = 3;  // FB_A
// Both threuil variables declared in variables.cpp
//const int PHTreuil    = 4;  // PH_TREUIL (direction)
//const int ENTreuil    = 5;  // EN_TREUIL (pwm)
const int SIGFlSensor   = 7;  // SIG_FL_SENSOR
const int STPEn         = 8;  // STP_EN
const int STPStep       = 9;  // STP_STEP
const int STPDir        = 10; // STP_DIR
const int HC12TXD       = 11;
const int HC12RXD       = 12;
const int PWMPompe      = 13; // PWM_POMPE
const int JOGPlateau    = 14;
const int RESET         = 15;
const int SVBout        = 16; // SV_BOUT
const int SVPurg        = 17; // SV_PURG
const int CSTreuil      = 18; // CSelect Treuil
const int STPFault      = 19; // Fault Plateau
const int SIGLSPlateau  = 20; // SIG_LS_PLATEAU
const int SIGLSTreuil   = 21; // SIG_LS_TREUIL


// ------- COMMAND NAMES -------
// This gives the command numbers a variable name that is easier to read
// But at compilation, the compiler replaces each instanfce of the variable names with the numbers we define it to be
#define TREUIL_UNROLL_CMD     8
#define TREUIL_ROLL_CMD       9
#define RETURN_MSG_CMD        10
#define TREUIL_UNROLL_MAN_CMD 12
#define TREUIL_ROLL_MAN_CMD   13
#define PUMP_CMD              30
#define TURN_TRAY_PURGE_CMD   40
#define PURGE_CMD             60
#define RESET_CMD             70
#define PURGE_NO_PUMP_CMD     80

// ------- VARIABLES -------
int position = 0;
int askedCommand = 0;
String HC12String;
int D = 0;
int N = 1;

float defaultNbTurns = 5; // Variable used to store a target number of winch turns to be used with PID
float freq = 0; // Flow meter frequence variable
float flow = 0; // Flow meter flow rate variable

// Declared in variables.cpp
//long positionEncTreuil  = -999; // Gives a value to encoder variable
//double Setpoint, Input, Output; // PID variables
//float pulseByTurn = 2398.31; // Number of pulses by turn of the winch (treuil)
double Kp=0.5, Ki=0.1, Kd=0; // Put Kd at 0.1 if we want to slow down and arrive smoothly at target

volatile int flowPulseCount = 0; // Volatile variables are better suited for use with interrupts
unsigned long previousMicros = 0; //variables pour gérer le temps
unsigned long currentMicros = 0;
unsigned long interval = 0;


// ------- OBJECTS -------
SoftwareSerial HC12(HC12TXD, HC12RXD); // HC-12 TX Pin, HC-12 RX Pin
Encoder encTreuil(ENCFBA, ENCFBB); //encA / encB
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
  pinMode(PHTreuil, OUTPUT);     
  pinMode(ENTreuil, OUTPUT);    

  // VALVE pin initiate
  pinMode(SVBout, OUTPUT);
  pinMode(SVPurg, OUTPUT);

  // STEPPER BOTTLES control pin initiale
  pinMode(STPStep, OUTPUT); // STEP PIN
  pinMode(STPDir, OUTPUT);  // DIR PIN
  pinMode(STPEn, OUTPUT);
  digitalWrite(STPEn,HIGH); // Disable stepper motor control
  //pinMode(stepHomeSwitch, INPUT_PULLUP);

  // PUMP control pin initiate
  pinMode(PWMPompe, OUTPUT);
 
  // --- LED ---
  pinMode(LED_BUILTIN, OUTPUT);

  // FLOW METER
  attachInterrupt(digitalPinToInterrupt(SIGFlSensor), pulse, RISING);

  // SWITCHES
  pinMode(SIGLSPlateau, INPUT); // SIG_LS_PLATEAU
  pinMode(SIGLSTreuil, INPUT);  // SIG_LS_TREUIL
  pinMode(JOGPlateau, INPUT_PULLUP); 

  // PREPARING MACHINE
  pumpOff(PWMPompe);
  valveIn(HC12, HC12String);
  delay(500);
  valvePurgeIn(SVPurg, SVBout);
  delay(4000);

  Serial.println("INITIALISATION");
  digitalWrite(STPEn,LOW); //Enable stepper motor control
  delay(1);
  homing(&position, STPDir, STPStep, SIGLSPlateau, STPEn, HC12, HC12String);
 
}

void loop() 
{
  askedCommand = checkCommunication(HC12, HC12String);
  delay(1);
  if(askedCommand==RESET_CMD)
  {
    Serial.println(askedCommand);
    returnMessage(HC12, 1);
    Resetfct();
  }
          
  else if(stop_loop==false)
  {

    switch (askedCommand){
      // Manual filling for bottles 1 to 6
      case 1: case 2: case 3: case 4: case 5: case 6:
        Serial.println(askedCommand);
        delay(2000);
        D = checkCommunication(HC12, HC12String);
        commandeRemplissageManuel(askedCommand, D);
        returnMessage(HC12, 1);
        D = 0;
        break;

      case TREUIL_UNROLL_CMD:
        treuilUnroll(defaultNbTurns, myPID, encTreuil, SIGLSTreuil, HC12, HC12String);
        returnMessage(HC12, 1);
        break;

      case TREUIL_ROLL_CMD:
        treuilRoll(defaultNbTurns, myPID, encTreuil, SIGLSTreuil, HC12, HC12String);
        Serial.println(stop_loop);
        returnMessage(HC12, 1);
        break;
      
      case RETURN_MSG_CMD:
        returnMessage(HC12, 1);
        break;

      case TREUIL_UNROLL_MAN_CMD:
        int debut_unroll;
        int fin_unroll;
        Serial.println(askedCommand);
        N=0;
        debut_unroll=millis();
        while(N==0)
        {
          N = checkCommunication(HC12, HC12String);
          delay(10);
        }
        fin_unroll=millis();
        Serial.println(N);
        Serial.println(fin_unroll-debut_unroll);
        treuilUnrollManual(N, 1, SIGLSTreuil, 1, HC12, HC12String);
        returnMessage(HC12, 1);
        break;

      case TREUIL_ROLL_MAN_CMD:
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
        treuilRollManual(N, 1, SIGLSTreuil, 1, HC12, HC12String);
        returnMessage(HC12, 1);
        break;
      
      // Fill bottles 1 to 6
      case 21: case 22: case 23: case 24: case 25: case 26:
        Serial.println(askedCommand);
        delay(2000);
        D = checkCommunication(HC12, HC12String);
        remplissage(askedCommand-20, D);
        returnMessage(HC12, 1);
        D = 0;
        break;

      // Turn tray to bottle position 1 to 6
      case 51: case 52: case 53: case 54: case 55: case 56:
        Serial.println(askedCommand);
        tournerPlateau(askedCommand-50);
        returnMessage(HC12, 1);
        break;
      
      case PUMP_CMD:
        Serial.println(askedCommand);
        delay(2000);
        D = checkCommunication(HC12, HC12String);
        Serial.println(D);
        pompage(D);
        returnMessage(HC12, 1);
        D = 0;
        break;
      
      case TURN_TRAY_PURGE_CMD:
        Serial.println(askedCommand);
        tournerPlateauPurge();
        returnMessage(HC12, 1);
        break;

      case PURGE_CMD:
        Serial.println(askedCommand);
        digitalWrite(STPEn, LOW); // Enable stepper motor control
        valvePurge();
        returnMessage(HC12, 1);
        break;
      
      case RESET_CMD:
        Serial.println(askedCommand);
        returnMessage(HC12, 1);
        Resetfct();
        break;

      case PURGE_NO_PUMP_CMD:
        Serial.println(askedCommand);
        returnMessage(HC12, 1);
        valvePurgeSansPompage();
        break;
      
      default: 
        if(digitalRead(JOGPlateau) == 0)
        {
          delay(5);
          if(digitalRead(JOGPlateau) == 0) // PROTECTION AU BRUIT
          {
            digitalWrite(STPEn, LOW); // Enable stepper motor control
            delay(1);
            manualTurning(14, 15, 100, JOGPlateau);
            digitalWrite(STPEn, HIGH); // Disable stepper motor control
          }
        }
        break;
    }
  }
}

// ----------- FONCTION--------------

void pompage(int duree)
{
  int debut;
  int fin;
  int d=0;
    pumpOn(PWMPompe);
    debut=millis();
    for (int i=0; i<5;i++)
    {
      d=checkCommunication(HC12, HC12String);
      Serial.println(d);
      if(d==100)
      {
        pumpOff(PWMPompe);
        Serial.println("arret d'urgence");
        stop_loop=true;
        return;
      }
      else
      {    
       delay(1000); 
      }
    }
    fin=millis();
    Serial.println(fin-debut);

}
 
void remplissage (int wantedBottle, int duree)
{
  digitalWrite(STPEn, LOW); // Enable stepper motor control
  delay(1);
  //PURGE
  //valvePurge();
  Serial.println(positionPlateau);

  // REMPLISSAGE
  //homing(&position, STPDir, STPStep, SIGLSPlateau,STPEn, HC12, HC12String);
  if(stop_loop==true){return;}
  delay(50);
  takePosition(&position, wantedBottle, STPDir, STPStep, STPEn, HC12, HC12String);
  if(stop_loop==true){return;}
  valveOut(HC12, HC12String);
  if(stop_loop==true){return;}
  pompage(duree);
  if(stop_loop==true){return;}
  valveIn(HC12, HC12String);

  digitalWrite(STPEn,HIGH); // Disable stepper motor control
}


void commandeRemplissageManuel(int wantedBottle, int duree)
{
  treuilUnroll(defaultNbTurns, myPID, encTreuil, SIGLSTreuil, HC12, HC12String);
  if(stop_loop==true){return;}
  digitalWrite(STPEn,LOW); // Enable stepper motor control
  delay(1);
 // homing(&position, STPDir, STPStep, SIGLSPlateau, STPEn,  HC12, HC12String);
  if(stop_loop==true){return;}
  delay(50);

  //PURGE

  valvePurge();
  Serial.println(positionPlateau);

  // REMPLISSAGE
  takePosition(&position, wantedBottle, STPDir, STPStep, STPEn, HC12, HC12String);
  Serial.println(positionPlateau);

  if(stop_loop==true){return;}
  valveOut(HC12, HC12String);
  if(stop_loop==true){return;}
  pompage(duree);
  if(stop_loop==true){return;}
  valveIn(HC12, HC12String);
  if(stop_loop==true){return;}
  digitalWrite(STPEn,HIGH); // Disable stepper motor control
  treuilRoll(defaultNbTurns, myPID, encTreuil, SIGLSTreuil, HC12, HC12String);

}

void tournerPlateau(int wantedBottle)
{
  digitalWrite(STPEn,LOW); // Enable stepper motor control
  delay(1);
  //homing(&position, STPDir, STPStep, SIGLSPlateau, STPEn, HC12, HC12String);
  Serial.println(position);
  if(stop_loop==true){return;}
  delay(50);
  takePosition(&position, wantedBottle, STPDir, STPStep, STPEn, HC12, HC12String);
  Serial.println(position);
  Serial.println(positionPlateau);
}


void tournerPlateauPurge()
{
  digitalWrite(STPEn, LOW); // Enable stepper motor control
  delay(1);
  Serial.println(position);
  if(stop_loop==true){return;}
  delay(50);
  takePurgePosition(&position, STPDir, STPStep, STPEn, HC12, HC12String);
}


void valvePurge()
{
  takePurgePosition(&position, STPDir, STPStep, STPEn, HC12, HC12String);
  if(stop_loop==true){return;}
  valvePurgeOut(SVPurg, SVBout);
  delay(4000);
  valveOut(HC12, HC12String);
  if(stop_loop==true){return;}
  pompage(8);
  if(stop_loop==true){return;}
  valveIn(HC12, HC12String);
  if(stop_loop==true){return;}
  delay(500);
  valvePurgeIn(SVPurg, SVBout);
  delay(4000);
}


void valvePurgeSansPompage()
{
  digitalWrite(STPEn,LOW); // Enable stepper motor control
  takePurgePosition(&position, STPDir, STPStep, STPEn, HC12, HC12String);
  if(stop_loop==true){return;}
  valvePurgeOut(SVPurg, SVBout);
  delay(4000);
  valveOut(HC12, HC12String);
  if(stop_loop==true){return;}
  delay(5000);
  if(stop_loop==true){return;}
  valveIn(HC12, HC12String);
  if(stop_loop==true){return;}
  delay(500);
  valvePurgeIn(SVPurg, SVBout);
  delay(4000);

}

//ISR pour le débimètre:
void pulse() {
  flowPulseCount++;
  previousMicros = currentMicros; // Update time since last pulse
}


void Resetfct()
{
    digitalWrite(RESET, LOW);

}
