#ifndef VALVE_PURGE_hpp
#define VALVE_PURGE_hpp

void valvePurgeActivate(int SVPurg_, int SVBout_);
void valveBoutActivate(int SVPurg_, int SVBout_);
void valveDeactivate(int SVPurg_, int SVBout_);
//void valveIn(SoftwareSerial &HC12object, String HC12String_);
//void valveOut(SoftwareSerial &HC12object, String HC12String_);


//ajouter SoftwareSerial &HC12object, String HC12String_ dans les paramètres pour l'arrêt d'urgence. 

#endif
