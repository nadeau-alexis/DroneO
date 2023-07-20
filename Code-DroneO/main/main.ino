// ---------- Main.ino ----------

#include <Arduino.h>
#include <SoftwareSerial.h>

#include "BDC_CONTROLER.hpp"
#include "STEPPER_CONTROLER.hpp"
#include "PUMP_CONTROLER.hpp"
#include "variables.hpp"
#include "HC12.hpp"
#include "string.h"
#include "VERIN_PURGE.hpp"


// ---------- CONSTANTS ----------
const int stepPin = 15;
const int dirPin = 14;
const int stepHomeSwitch = 16;  //SP
const int irsensor = 17;  //SJ
const int boutonManuel = 18;
const int stepperEnable = 23;  
const int SwitchTreuilFinE = 20;  //SE
const int SwitchTreuilFinD = 21;  //SD

const int verinOutPin = 28;
const int verinInPin = 29;

const int pump1Pin = 48;
const int pump2Pin =49;
const int ResetPin = 22;

// ---------- VARIABLES ----------
int position = 0;
int askedCommand = 0;
bool modeNormal = false;
String HC12String;
int D = 0;
int N = 1;

// ----------- OBJETS ------------
SoftwareSerial HC12(11, 12); // HC-12 TX Pin, HC-12 RX Pin


// --------- PROTOTYPES -----------

void commandeRemplissageManuel(int wantedBottle, int duree);
void remplissage (int wantedBottle, int duree);
//--------------------------------
void setup() 
{
  Serial.begin(9600);
  HC12.begin(9600);

  digitalWrite(ResetPin, HIGH);
  delay(200); 
  pinMode(ResetPin, OUTPUT);
  // ---------- pin initialisation ----------
  // --- Motors ---
  // Motor_1 TREUIL control pin initiate
  pinMode(4, OUTPUT);     
  pinMode(5, OUTPUT);    
  pinMode(9, OUTPUT); // Speed control
  
  // Motor_2 VERIN control pin initiate
  pinMode(7, OUTPUT);     
  pinMode(8, OUTPUT);    
  pinMode(10, OUTPUT);  // Speed control

  // VERIN Purge control pin initiate
  pinMode(verinOutPin, OUTPUT);     
  pinMode(verinInPin, OUTPUT);    
  
  //Disable the Motor Shield output
  pinMode(6, OUTPUT); 
  digitalWrite(6, LOW); 

  // STEPPER BOTTLES control pin initiale
  pinMode(stepPin, OUTPUT); // STEP PIN
  pinMode(dirPin, OUTPUT); // DIR PIN
  pinMode(stepperEnable, OUTPUT);
  digitalWrite(stepperEnable,HIGH);//Disable stepper motor control
  pinMode(stepHomeSwitch, INPUT_PULLUP);

  // PUMP 1 AND PUMP 2 control pin initiate
  pinMode(pump1Pin, OUTPUT);
  pinMode(pump2Pin, OUTPUT);
  // --- LED ---
  pinMode(LED_BUILTIN, OUTPUT);

  // SWITCHES
  pinMode(SwitchTreuilFinE, INPUT_PULLUP); //SE
  pinMode(SwitchTreuilFinD, INPUT_PULLUP); //SD1
  pinMode(irsensor, INPUT_PULLUP); //SIR
  pinMode(boutonManuel, INPUT_PULLUP); 

  // PREPARING MACHINE

  pump2off(pump2Pin);
  pump1off(pump1Pin);
  verinIn(HC12, HC12String);
  delay(500);
  verinpurgeIn(verinOutPin, verinInPin);
  delay(4000);

  Serial.println("INITIALISATION");
  digitalWrite(stepperEnable,LOW);//Enable stepper motor control
  delay(1);
  homing(&position, dirPin, stepPin, stepHomeSwitch, stepperEnable, HC12, HC12String);
 
 
}

void loop() {
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
            treuilUnroll(1,SwitchTreuilFinD, HC12, HC12String);
            returnMessage(HC12, 1);
          }
        
          else if(askedCommand==9)
          {
            treuilRoll(1,SwitchTreuilFinE, HC12, HC12String);
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
            treuilUnrollManual(N, 1,SwitchTreuilFinD, irsensor, HC12, HC12String);
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
            treuilRollManual(N, 1, SwitchTreuilFinE, irsensor, HC12, HC12String);
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
            tournerplateau(askedCommand-50);
            returnMessage(HC12, 1);
          }
        else if(askedCommand==30)
          {
            Serial.println(askedCommand);
            delay(2000);
            D = checkCommunication(HC12, HC12String);
            Serial.println(D);
            Pompage(D);
            returnMessage(HC12, 1);
            D = 0;
          }
          else if(askedCommand==40)
          {
            Serial.println(askedCommand);
            tournerplateaupurge();
            returnMessage(HC12, 1);
          }
          else if(askedCommand==60)
          {
            Serial.println(askedCommand);
            digitalWrite(stepperEnable,LOW);//Enable stepper motor control
            verinpurge();
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
            verinpurgesanspompage();
          }
         else if(digitalRead(boutonManuel) == 0)
          {
            delay(5);
            if(digitalRead(boutonManuel) == 0)//PROTECTION AU BRUIT
            {
              digitalWrite(stepperEnable,LOW);//Enable stepper motor control
              delay(1);
              manualTurning(14, 15, 100, boutonManuel);
              digitalWrite(stepperEnable,HIGH);//Disable stepper motor control
            }
          }
  }
}

// ----------- FONCTION--------------

void Pompage(int duree)
 {
  int debut1, debut2;
  int fin1, fin2;
  int d1=0;
  int d2=0;
    pump1on(pump1Pin);
    debut1=millis();
    for (int i=0; i<5;i++)
    {
      d1=checkCommunication(HC12, HC12String);
      Serial.println(d1);
      if(d1==100)
      {
        pump1off(pump1Pin);
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
    Serial.println(fin1-debut1);
    pump2on(pump2Pin);
    debut2=millis();

    for (int i=0; i<duree;i++)
    {
      d2=checkCommunication(HC12, HC12String);
      Serial.println(d2);
      if(d2==100)
      {
        pump2off(pump2Pin);
        pump1off(pump1Pin);
        stop_loop=true;
        Serial.println("arret d'urgence");
        return;
        
      }
      else
      {delay(1000);}
    }
    pump2off(pump2Pin);
    pump1off(pump1Pin);
    fin2=millis();
    Serial.println(fin1-debut1+fin2-debut2);

 }
 
void remplissage (int wantedBottle, int duree)
{
  digitalWrite(stepperEnable,LOW);//Enable stepper motor control
  delay(1);
  //PURGE
  //verinpurge();
  Serial.println(positionplateau);

  // REMPLISSAGE
  //homing(&position, dirPin, stepPin, stepHomeSwitch,stepperEnable, HC12, HC12String);
  if(stop_loop==true){return;}
  delay(50);
  takePosition(&position, wantedBottle, dirPin, stepPin, stepperEnable, HC12, HC12String);
  if(stop_loop==true){return;}
  verinOut(HC12, HC12String);
  if(stop_loop==true){return;}
  Pompage(duree);
  if(stop_loop==true){return;}
  verinIn(HC12, HC12String);

  digitalWrite(stepperEnable,HIGH);//Disable stepper motor control
}


void commandeRemplissageManuel(int wantedBottle, int duree)
{
  treuilUnroll(1,SwitchTreuilFinD,HC12, HC12String);
  if(stop_loop==true){return;}
  digitalWrite(stepperEnable,LOW);//Enable stepper motor control
  delay(1);
 // homing(&position, dirPin, stepPin, stepHomeSwitch, stepperEnable,  HC12, HC12String);
  if(stop_loop==true){return;}
  delay(50);

  //PURGE

  verinpurge();
  Serial.println(positionplateau);

  // REMPLISSAGE
  takePosition(&position, wantedBottle, dirPin, stepPin, stepperEnable, HC12, HC12String);
  Serial.println(positionplateau);

  if(stop_loop==true){return;}
  verinOut(HC12, HC12String);
  if(stop_loop==true){return;}
  Pompage(duree);
  if(stop_loop==true){return;}
  verinIn(HC12, HC12String);
  if(stop_loop==true){return;}
  digitalWrite(stepperEnable,HIGH);//Disable stepper motor control
  treuilRoll(1,SwitchTreuilFinE, HC12, HC12String);
}

void tournerplateau(int wantedBottle)
{
  digitalWrite(stepperEnable,LOW);//Enable stepper motor control
  delay(1);
  //homing(&position, dirPin, stepPin, stepHomeSwitch, stepperEnable, HC12, HC12String);
  Serial.println(position);
  if(stop_loop==true){return;}
  delay(50);
  takePosition(&position, wantedBottle, dirPin, stepPin, stepperEnable, HC12, HC12String);
  Serial.println(position);
  Serial.println(positionplateau);
}
void tournerplateaupurge()
{
  digitalWrite(stepperEnable,LOW);//Enable stepper motor control
  delay(1);
  Serial.println(position);
  if(stop_loop==true){return;}
  delay(50);
  takePurgePosition(&position, dirPin, stepPin, stepperEnable, HC12, HC12String);
 }

void verinpurge()
{
  takePurgePosition(&position,dirPin, stepPin, stepperEnable, HC12, HC12String);
  if(stop_loop==true){return;}
  verinpurgeOut(verinOutPin, verinInPin);
  delay(4000);
  verinOut(HC12, HC12String);
  if(stop_loop==true){return;}
  Pompage(8);
  if(stop_loop==true){return;}
  verinIn(HC12, HC12String);
  if(stop_loop==true){return;}
  delay(500);
  verinpurgeIn(verinOutPin, verinInPin);
  delay(4000);
}



void verinpurgesanspompage()
{
  digitalWrite(stepperEnable,LOW);//Enable stepper motor control
  takePurgePosition(&position,dirPin, stepPin, stepperEnable, HC12, HC12String);
  if(stop_loop==true){return;}
  verinpurgeOut(verinOutPin, verinInPin);
  delay(4000);
  verinOut(HC12, HC12String);
  if(stop_loop==true){return;}
  delay(5000);
  if(stop_loop==true){return;}
  verinIn(HC12, HC12String);
  if(stop_loop==true){return;}
  delay(500);
  verinpurgeIn(verinOutPin, verinInPin);
  delay(4000);

}


void Resetfct()
{
    digitalWrite(ResetPin, LOW);

  }
