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
const int PWMPompe = 1;

const int CSTreuil = 4;  // CSelect Treuil
const int ENTreuil = 27;  // Enable Treuil
const int PHTreuil = 26;  // Enable Treuil
const int ENCFBA = 25;  //encodeur du Treuil FB_A
const int ENCFBB = 24;  //encodeur du Treuil FB_B

const int SIGLSTreuil = 7;  //anciennement SwitchTreuilFinE


const int STPFault = 5;  // Fault Plateau
const int SIGLSPlateau = 6;  //anciennement stepHomeSwitch
const int STPEn = 30; //anciennement stepperEnable 
const int STPStep = 31; //anciennement stepPin
const int STPDir = 32; //anciennement dirPin
const int JOGPlateau = 16; //anciennement boutonManuel

const int SVPurg = 19; //ouverture valve pour purge
const int SVBout = 18; //ouverture valve pour remplissage bouteille

const int SIGFlSensor = 29; //mesure du débit

const int HC12RXD = 34;
const int HC12TXD = 33;

const int RESET = 17; //anciennementResetPin

// ---------- VARIABLES ----------
int position = 0;
int askedCommand = 0;
bool modeNormal = false;
String HC12String;
int D = 0;
int N = 1;

// ----------- OBJETS ------------
SoftwareSerial HC12(HC12TXD, HC12RXD); // HC-12 TX Pin, HC-12 RX Pin


// --------- PROTOTYPES -----------

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
  // Motor TREUIL control pin initiate
  
  pinMode(ENTreuil, OUTPUT); //    
  pinMode(CSTreuil, OUTPUT); //   
  pinMode(PHTreuil, OUTPUT); //
  
  pinMode(ENCFBA, INPUT_PULLUP); //
  pinMode(ENCFBB, INPUT_PULLUP); //
  pinMode(SIGLSTreuil, INPUT_PULLUP);

  digitalWrite(ENTreuil, LOW); //Disable treuil motor control (HIGH ou LOW à valider)


 // Motor Stepper Plateau control pin initiale
  
  pinMode(STPStep, OUTPUT); // STEP PIN
  pinMode(STPDir, OUTPUT); // DIR PIN
  pinMode(STPEn, OUTPUT);
  pinMode(SIGLSPlateau, INPUT_PULLUP);
  
  digitalWrite(STPEn, HIGH);// Disable stepper motor control (HIGH ou LOW à valider)
   
    // Valve control pin initiate
  
  pinMode(SVPurg, OUTPUT);     
  pinMode(SVBout, OUTPUT);    

  // Lecture débitmètre pin initiate
  pinMode(SIGFlSensor, INPUT_PULLUP);     

  // PUMP 1 AND PUMP 2 control pin initiate
  pinMode(PWMPompe, OUTPUT);
  

  // SWITCH
 
  pinMode(JOGPlateau, INPUT_PULLUP); 

  // PREPARING MACHINE

  //pump2off(pump2Pin);
  //pump1off(pump1Pin);
  //verinIn(HC12, HC12String);
  //delay(500);
  //verinpurgeIn(verinOutPin, verinInPin);
  //delay(4000);

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
