#include "BDC_CONTROLLER.hpp"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Encoder.h>
#include <PID_v1.h>
#include "HC12.hpp"
#include "variables.hpp"

void treuilRoll(int target_nbTurns, PID &PIDObject, Encoder &EncoderObject, int hallSensor, SoftwareSerial &HC12object, String HC12String_)
{
  Setpoint = target_nbTurns * pulseByTurn;
  long newEncTreuil;
  newEncTreuil = EncoderObject.read();
  digitalWrite(PHTreuil, HIGH);

  while(newEncTreuil < Setpoint - 2) 
  {
    newEncTreuil = EncoderObject.read();
    Input = EncoderObject.read();
    PIDObject.Compute();
    if(digitalRead(CSTreuil) >= 2.1)
    {
      emergencyUnroll(PIDObject, EncoderObject, hallSensor, HC12object, HC12String_);
    }
    else if(digitalRead(hallSensor) == HIGH || checkCommunication(HC12object, HC12String_)==100)
    {
      analogWrite(ENTreuil, 0);
      positionEncTreuil = newEncTreuil;
      Serial.println("arret d'urgence");
      stop_loop=true;
      return;
    }
    else
    {
      analogWrite(ENTreuil, Output);
    }
  }

  analogWrite(ENTreuil, 0); // STOP MOTOR

  if (newEncTreuil != positionEncTreuil) 
  {
    positionEncTreuil = newEncTreuil;
  }

}

void treuilUnroll(int target_nbTurns, PID &PIDObject, Encoder &EncoderObject, int hallSensor, SoftwareSerial &HC12object, String HC12String_)
{
  Setpoint = target_nbTurns * pulseByTurn;
  long newEncTreuil;
  newEncTreuil = EncoderObject.read();
  digitalWrite(PHTreuil, LOW);

  while(newEncTreuil > Setpoint - 2) 
  {
    newEncTreuil = EncoderObject.read();
    Input = EncoderObject.read();
    PIDObject.Compute();
    if(digitalRead(CSTreuil) >= 2.1)
    {
      emergencyRoll(PIDObject, EncoderObject, hallSensor, HC12object, HC12String_);
    }
    else if(digitalRead(hallSensor) == HIGH || checkCommunication(HC12object, HC12String_)==100)
    {
      analogWrite(ENTreuil, 0);
      positionEncTreuil = newEncTreuil;
      Serial.println("arret d'urgence");
      stop_loop=true;
      return;
    }
    else
    {
      analogWrite(ENTreuil, Output);
    }
  }

  analogWrite(ENTreuil, 0); // STOP MOTOR

  if (newEncTreuil != positionEncTreuil) 
  {
    positionEncTreuil = newEncTreuil;
  }

}

void emergencyRoll(PID &PIDObject, Encoder &EncoderObject, int hallSensor, SoftwareSerial &HC12object, String HC12String_)
{
  Setpoint = 2 * pulseByTurn;
  long newEncTreuil;
  newEncTreuil = EncoderObject.read();
  digitalWrite(PHTreuil, HIGH);

  while(newEncTreuil < Setpoint - 2) 
  {
    newEncTreuil = EncoderObject.read();
    Input = EncoderObject.read();
    PIDObject.Compute();
    if(digitalRead(hallSensor) == HIGH || checkCommunication(HC12object, HC12String_)==100)
    {
      analogWrite(ENTreuil, 0);
      positionEncTreuil = newEncTreuil;
      Serial.println("arret d'urgence");
      stop_loop=true;
      return;
    }
    else
    {
      analogWrite(ENTreuil, Output);
    }
  }

  stop_loop = false;
  analogWrite(ENTreuil, 0); // STOP MOTOR

  if (newEncTreuil != positionEncTreuil) 
  {
    positionEncTreuil = newEncTreuil;
  }

}

void emergencyUnroll(PID &PIDObject, Encoder &EncoderObject, int hallSensor, SoftwareSerial &HC12object, String HC12String_)
{
  Setpoint = 2 * pulseByTurn;
  long newEncTreuil;
  newEncTreuil = EncoderObject.read();
  digitalWrite(PHTreuil, LOW);

  while(newEncTreuil > Setpoint - 2) 
  {
    newEncTreuil = EncoderObject.read();
    Input = EncoderObject.read();
    PIDObject.Compute();
    if(digitalRead(hallSensor) == HIGH || checkCommunication(HC12object, HC12String_)==100)
    {
      analogWrite(ENTreuil, 0);
      positionEncTreuil = newEncTreuil;
      Serial.println("arret d'urgence");
      stop_loop=true;
      return;
    }
    else
    {
      analogWrite(ENTreuil, Output);
    }
  }

  stop_loop = false;
  analogWrite(ENTreuil, 0); // STOP MOTOR

  if (newEncTreuil != positionEncTreuil) 
  {
    positionEncTreuil = newEncTreuil;
  }

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