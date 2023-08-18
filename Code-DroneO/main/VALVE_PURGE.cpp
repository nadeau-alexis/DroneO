#include "VALVE_PURGE.hpp"
#include <Arduino.h>

void valvePurgeActivate(int SVPurg_, int SVBout_)
{
    digitalWrite(SVPurg_, HIGH);
	  digitalWrite(SVBout_, LOW);
}

void valveBoutActivate(int SVPurg_, int SVBout_)
{
    digitalWrite(SVPurg_, LOW);
	  digitalWrite(SVBout_, HIGH);
}

void valveDeactivate(int SVPurg_, int SVBout_)
{
    digitalWrite(SVPurg_, LOW);
    digitalWrite(SVBout_, LOW);
}

// void valveIn(SoftwareSerial &HC12object, String HC12String_)
// {
//   digitalWrite(8, LOW);
//   digitalWrite(7, HIGH);      // Set the rotation of motor
//   digitalWrite(6, HIGH);      // ENABLE

//   analogWrite(10, 255); // SET SPEED
//   for(int i=0;i<4;i++)
//   {
//       if(checkCommunication(HC12object, HC12String_)!=100)
//           { 
//             delay(500);
//           }
//           else
//           {
//             analogWrite(10, 0);         // STOP MOTOR
//             digitalWrite(6, LOW);
//             Serial.println("arret d'urgence");
//             stop_loop=true;
//             return;
//           }
//   }
  
//   analogWrite(10, 0);         // STOP MOTOR
//   digitalWrite(6, LOW);       // DISABLE
// }

// void valveOut(SoftwareSerial &HC12object, String HC12String_)
// {
//   digitalWrite(7, LOW);
//   digitalWrite(8, HIGH);      // Set the rotation of motor
//   digitalWrite(6, HIGH);      // ENABLE

//   analogWrite(10, 255);       // set the motor_2 speed ;
//  for(int i=0;i<4;i++)
//   {
//       if(checkCommunication(HC12object, HC12String_)!=100)
//           { 
//             delay(500);
//           }
//           else
//           {
//             analogWrite(10, 0);         // STOP MOTOR
//             digitalWrite(6, LOW);
//             Serial.println("arret d'urgence");
//             stop_loop=true;
//             return;
//           }
//   }
//   analogWrite(10, 0);         // set the motor_2 speed ;
//   digitalWrite(6, LOW);       // DISABLE
// }
