#include "BDC_CONTROLER.hpp"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "HC12.hpp"
#include "variables.hpp"




void treuilRoll(int speed, int limitswitch,SoftwareSerial &HC12object, String HC12String_)
{
  int i = 0;

  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);      // Set the rotation of motor
  digitalWrite(6, HIGH);      // ENABLE

  digitalWrite(9, HIGH);      // SET SPEED
  while (digitalRead(limitswitch) == LOW) {
    if(checkCommunication(HC12object, HC12String_)!=100)
       {
    delay(3);}
    else
    {
      digitalWrite(9, LOW);          // STOP MOTOR
      digitalWrite(6, LOW);
      Serial.println("arret d'urgence");
      stop_loop=true;
      return;
      }
  }

  delay(100);
  digitalWrite(9, LOW);          // STOP MOTOR
  digitalWrite(6, LOW);       // DISABLE
}
void treuilUnroll(int speed, int limitswitch, SoftwareSerial &HC12object, String HC12String_)
{
  int i = 0;
  digitalWrite(5, LOW);
  digitalWrite(4, HIGH);      // Set the rotation of motor
  digitalWrite(6, HIGH);      // ENABLE

  digitalWrite(9, HIGH);      // SET SPEED
  while (digitalRead(limitswitch) == LOW) 
  {
    if(checkCommunication(HC12object, HC12String_)!=100)
    {
        delay(3); //continuer le déroulement du treuil
    }
    else
    {
        digitalWrite(9, LOW);          // STOP MOTOR
        digitalWrite(6, LOW);
        Serial.println("arret d'urgence");
        stop_loop=true;
        return;
      }
  }
  delay(100);
  digitalWrite(9, LOW);          // STOP MOTOR
 digitalWrite(6, LOW);       // DISABLE
}

void treuilRollManual(int n, int speed, int limitswitch, int irsensor, SoftwareSerial &HC12object, String HC12String_)
{
  
    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);      // Set the rotation of motor
    digitalWrite(6, HIGH);      // ENABLE
    digitalWrite(9, HIGH);      // SET SPEED
	for(int j = 0; j<n ; j++)
	{
    if (digitalRead(limitswitch) == 1)
    {
        j=25;
    }
    else
    {
        Serial.println(j+1);
        while (digitalRead(irsensor) == 0) //obstacle detecté
        {
         delay(3);      
        } 
        while (digitalRead(irsensor) == 1)//obstacle non detecté
        {
           if(checkCommunication(HC12object, HC12String_)!=100)
           {
            delay(3);
           }
           else
           {
              digitalWrite(9, LOW);          // STOP MOTOR
              digitalWrite(6, LOW);
              Serial.println("arret d'urgence");
              stop_loop=true;
              return;
            }        
        }   
     }  
  }
	
	delay(50);
	digitalWrite(9, LOW);          // STOP MOTOR
	digitalWrite(6, LOW);       // DISABLE
}

void treuilUnrollManual(int n, int speed, int limitswitch, int irsensor ,SoftwareSerial &HC12object, String HC12String_)
{
 
	  digitalWrite(4, HIGH); 
    digitalWrite(5, LOW);       // Set the rotation of motor
    digitalWrite(6, HIGH);      // ENABLE
    digitalWrite(9, HIGH);      // SET SPEED
    for(int j=0;j<n;j++)
    {
      if (digitalRead(limitswitch) == 1)
      {
        j=25;
       }
       else
       {
         Serial.println(j+1);
         while (digitalRead(irsensor) == 0) //obstacle detecté
         {
        	delay(3);		  
         } 
         while (digitalRead(irsensor) == 1)//obstacle non detecté
         {
        		if(checkCommunication(HC12object, HC12String_)!=100)
            {
              delay(3);
            }
              else
              {
                digitalWrite(9, LOW);          // STOP MOTOR
                digitalWrite(6, LOW);
                Serial.println("arret d'urgence");
                stop_loop=true;
                return;
              }   	  
       	  }
       }      
    }
  digitalWrite(9, LOW);          // STOP MOTOR
  digitalWrite(6, LOW);       // DISABLE
}




void verinIn(SoftwareSerial &HC12object, String HC12String_)
{
  digitalWrite(8, LOW);
  digitalWrite(7, HIGH);      // Set the rotation of motor
  digitalWrite(6, HIGH);      // ENABLE

  analogWrite(10, 255); // SET SPEED
  for(int i=0;i<4;i++)
  {
      if(checkCommunication(HC12object, HC12String_)!=100)
          { 
            delay(500);
          }
          else
          {
            analogWrite(10, 0);         // STOP MOTOR
            digitalWrite(6, LOW);
            Serial.println("arret d'urgence");
            stop_loop=true;
            return;
          }
  }
  
  analogWrite(10, 0);         // STOP MOTOR
  digitalWrite(6, LOW);       // DISABLE
}

void verinOut(SoftwareSerial &HC12object, String HC12String_)
{
  digitalWrite(7, LOW);
  digitalWrite(8, HIGH);      // Set the rotation of motor
  digitalWrite(6, HIGH);      // ENABLE

  analogWrite(10, 255);       // set the motor_2 speed ;
 for(int i=0;i<4;i++)
  {
      if(checkCommunication(HC12object, HC12String_)!=100)
          { 
            delay(500);
          }
          else
          {
            analogWrite(10, 0);         // STOP MOTOR
            digitalWrite(6, LOW);
            Serial.println("arret d'urgence");
            stop_loop=true;
            return;
          }
  }
  analogWrite(10, 0);         // set the motor_2 speed ;
  digitalWrite(6, LOW);       // DISABLE
}
