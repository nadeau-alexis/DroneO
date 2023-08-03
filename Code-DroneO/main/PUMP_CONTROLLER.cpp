#include "PUMP_CONTROLLER.hpp"
#include <Arduino.h>

void pumpProto (int PWMPompe_)
{
    float volume = 0;
    while(volume < targetVolume - 100) 
    {
        analogWrite(PWMPompe_, 255);
        volume = flowMeter();
    }

    while(volume <= targetVolume - 5)
    {
        analogWrite(PWMPompe_, 75);
        volume = flowMeter();
    }

    digitalWrite(PWMPompe_, LOW); // Stop pump

}

float flowMeter()
{
    float volume;
    unsigned long interval;
    float freq;
    float flow; 
    currentMicros = micros(); //on enregistre le nombre de microsecondes depuis le début du programme
    interval = currentMicros - previousMicros; //interval entre deux pulses (voir interrupt)
    freq = 1.0/(float(interval)/1000000.0); //On calcule la fréquence
    flow = freq/21.0; //On calcule le débit à partir de la formule fournie par la datasheet du débitmètre
    volume = flow * interval;

    Serial.print("currentMicros: ");
    Serial.print(currentMicros);
    Serial.print("/ interval: ");
    Serial.print(interval);
    Serial.print(" pulse: ");
    Serial.print(flowPulseCount);
    Serial.print("/ frequency: ");
    Serial.print(freq);
    Serial.print("/ flow: ");
    Serial.println(flow);
    Serial.println("/ volume: ");
    Serial.println(volume);
    
    return volume;
}

void pumpOn(int PWMPompe_)
{
    digitalWrite(PWMPompe_, HIGH);
}

void pumpOff(int PWMPompe_)
{
    digitalWrite(PWMPompe_, LOW);
}

//ISR pour le débimètre:
void pulse() {
  flowPulseCount++;
  previousMicros = currentMicros; // Update time since last pulse
}