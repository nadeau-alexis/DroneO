#include "BDC_CONTROLLER.hpp"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Encoder.h>
#include <PID_v1.h>
#include "HC12.hpp"
#include "variables.hpp"

// void prototypeFunction(int target_nbTurns, PID &PIDObject, Encoder &EncoderObject, int hallSensor, SoftwareSerial &HC12object, String HC12String_)
// {
//   Setpoint = target_nbTurns * pulseByTurn;
//   long newEncTreuil;
//   newEncTreuil = EncoderObject.read();
//   Input = EncoderObject.read();
//   PIDObject.Compute();
  
//   if (newEncTreuil < Setpoint - 2) {
//     digitalWrite(PHTreuil, HIGH);
//     analogWrite(ENTreuil, Output);
//   }
//   else if (newEncTreuil > Setpoint + 2) {
//     digitalWrite(PHTreuil, LOW);
//     analogWrite(ENTreuil, Output);
//   }
//   else {
//     analogWrite(ENTreuil, 0);
//   }

//   if (newEncTreuil != positionEncTreuil) {
//     positionEncTreuil = newEncTreuil;
//   }

//   Serial.print("Encoder = ");
//   Serial.print(newEncTreuil);
//   Serial.print(" / Output = ");
//   Serial.print(Output);
//   Serial.println();

// }

void treuilRoll(int target_nbTurns, PID &PIDObject, Encoder &EncoderObject, int hallSensor, SoftwareSerial &HC12object, String HC12String_)
{
  Setpoint = target_nbTurns * pulseByTurn;
  long newEncTreuil;
  newEncTreuil = EncoderObject.read();
  Input = EncoderObject.read();
  PIDObject.Compute();
  digitalWrite(PHTreuil, HIGH);

  while(newEncTreuil < Setpoint - 2) {
    if(digitalRead(hallSensor) == HIGH || checkCommunication(HC12object, HC12String_)==100)
    {
      analogWrite(ENTreuil, 0);
      Serial.println("arret d'urgence");
      stop_loop=true;
      return;
    }
    else
    {
      analogWrite(ENTreuil, Output);
    }
  }

  analogWrite(enTreuilPin, 0); // STOP MOTOR

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
  Input = EncoderObject.read();
  PIDObject.Compute();
  digitalWrite(PHTreuil, LOW);

  while(newEncTreuil > Setpoint - 2) {
    if(digitalRead(hallSensor) == HIGH || checkCommunication(HC12object, HC12String_)==100)
    {
      analogWrite(ENTreuil, 0);
      Serial.println("arret d'urgence");
      stop_loop=true;
      return;
    }
    else
    {
      analogWrite(ENTreuil, Output);
    }
  }

  analogWrite(enTreuilPin, 0); // STOP MOTOR

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




void valveIn(SoftwareSerial &HC12object, String HC12String_)
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

void valveOut(SoftwareSerial &HC12object, String HC12String_)
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
