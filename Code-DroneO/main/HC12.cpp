#include "HC12.hpp"
#include <Arduino.h>

int checkCommunication(SoftwareSerial &HC12object, String HC12String_)
{   
    if(HC12object.available())
    {   
        //Serial.println("available");
        digitalWrite(commLedPin, HIGH);
        int buffer = 50;
        char HC12message[buffer];
        char *HC12ParsedMessage;
        char delim[] = "/";

        while(HC12object.available())
        {
            HC12String_ = HC12object.readString();
            HC12String_.toCharArray(HC12message, buffer);
            HC12ParsedMessage = strtok(HC12message,delim);
            HC12ParsedMessage = strtok(NULL,delim);
        }
        
        int returnValue = 0;
        sscanf(HC12ParsedMessage, "%d", &returnValue);
        //returnValue = atoi(HC12ParsedMessage);
        digitalWrite(commLedPin, LOW);
        return returnValue;
    }
    digitalWrite(commLedPin, LOW);
    return 0;
}

//0 for error, 1 for success
void returnMessage(SoftwareSerial &HC12object, int returnMessageNumber)
{
    if(returnMessageNumber==1)
    {
        Serial.println("returnMessage");
        HC12object.write("BUFFER,BUFFER,BUFFER/1");      // Send that data to HC-12
        //HC12object.flush();
        //HC12object.end();
    }
    else
    {
        HC12object.write("BUFFER,BUFFER,BUFFER/0");      // Send that data to HC-12
        //HC12object.flush();
        //HC12object.end();
    }
}
