/*
Ce programme a pour but de tester le premier prototype du système de contrôle du Drone-O

Raphaël Pereira

*/

//Inclusion des librairies
#include <AccelStepper.h> //Pour contrôler le Plateau
#include <Encoder.h> //Pour contrôler le Treuil
#include <PID_v1.h> //Pour contrôler le Treuil


//Déclaration des pins
const int SVBout = 16; //SV_BOUT
const int svPurgPin = 17; //SV_PURG
const int pompePin = 13; //PWM_POMPE
const int PHTreuil = 4; //EN_TREUIL (pwm)
const int ENTreuil = 5; //PH_TREUIL (direction)
const int SIGFlSensor = 7; //SIG_FL_SENSOR
const int lsPlateauPin = 20; //SIG_LS_PLATEAU
const int lsTreuilPin = 21; //SIG_LS_TREUIL
const int ledPin = 6; //TEMPORAIRE Led d'état

//Déclaration des variables
float nbTours = 5;
float pulseParTour = 2398.31; //nombre de pulses par tour du treuil
float freq = 0; //variable pour stocker la fréquence
float flow = 0; //variable pour stocker le débit

long positionEncTreuil  = -999; //Donne une valeur à la variable de l'encodeur

double Setpoint, Input, Output; ////Variables pour le PID
double Kp=0.5, Ki=0.1, Kd=0; //mettre Kd à 0.1 si on veut ralentir avant d'atteindre la valeur

volatile int x = 0; //on utilise une variable volatile car mieux adaptée aux interrupts

bool lsPlateauState = false;
bool lsTreuilState = false;

unsigned long previousMicros = 0; //variables pour gérer le temps
unsigned long currentMicros = 0;
unsigned long interval = 0;

//Déclaration des objets
AccelStepper stepper1(1, 9, 10); // (Type of driver: with 2 pins, STEP, DIR), Best Performance: both pins have interrupt capability
Encoder encTreuil(3, 2); //encA / encB
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//Déclaration des fonctions
void testValves();
void testPompe();
void testStepper();
void testTreuil();
void testWaterFlow();
void testHallSensor();

void setup() {
  // put your setup code here, to run once:
  
  pinMode(SVBout, OUTPUT);
  pinMode(svPurgPin, OUTPUT);
  pinMode(pompePin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(ENTreuil, OUTPUT);
  pinMode(PHTreuil, OUTPUT);
  pinMode(lsPlateauPin, INPUT);
  pinMode(lsTreuilPin, INPUT);

  stepper1.setMaxSpeed(3200); // Set maximum speed value for the stepper
  stepper1.setAcceleration(500); // Set acceleration value for the stepper
  stepper1.setCurrentPosition(0); // Set the current position to 0 steps
  stepper1.setEnablePin(8); //EN (on déclare ici que notre driver a un pin ENABLE)
  stepper1.setPinsInverted(0,0,1); //On inverse le pin ENABLE (DIR, STEP, EN)

  //On initialise les variables du PID pour le Treuil
  Input = 0;
  Setpoint = nbTours * pulseParTour;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  attachInterrupt(digitalPinToInterrupt(SIGFlSensor), pulse, RISING); //on active les interrupts sur le pin du débitmètre

  //Utile pour débug
  Serial.begin(9600);
  
  //Simplement pour informer que le programme est initialisé
  delay(1000);

  for(int i = 0; i<3; i++) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  /* Fonctions de test des composants individuels*/
  //testValves();
  //testPompe();
  //testStepper();
  //testTreuil();
  //testWaterFlow();
  //testHallSensor();

  /* Fonction de test générale, chaque composant est démarré en séquence puis éteint */
  /*
  digitalWrite(ledPin, HIGH);
  delay(5000);
  digitalWrite(SVBout, HIGH);
  delay(5000);
  digitalWrite(svPurgPin, HIGH);
  delay(5000);
  digitalWrite(pompePin, HIGH);
  delay(5000);
  stepper1.enableOutputs();
  stepper1.setSpeed(3200);
  stepper1.runSpeed();
  delay(5000);
  digitalWrite(PHTreuil, HIGH);
  analogWrite(ENTreuil, 255);

  delay(10000);

  digitalWrite(ledPin, LOW);
  digitalWrite(SVBout, LOW);
  digitalWrite(svPurgPin, LOW);
  digitalWrite(pompePin, LOW);
  stepper1.disableOutputs();
  stepper1.setSpeed(0);
  analogWrite(ENTreuil, 0);
  
  delay(10000);
  */
}

void testValves() {

  digitalWrite(SVBout, HIGH);
  digitalWrite(ledPin, HIGH);
  delay(5000);
  digitalWrite(SVBout, LOW);
  digitalWrite(ledPin, LOW);
  delay(1000);
  digitalWrite(svPurgPin, HIGH);
  digitalWrite(ledPin, HIGH);
  delay(5000);
  digitalWrite(svPurgPin, LOW);
  digitalWrite(ledPin, LOW);
  delay(1000);

}

void testPompe() {

  digitalWrite(pompePin, HIGH);
  digitalWrite(ledPin, HIGH);
  delay(5000);
  digitalWrite(pompePin, LOW);
  digitalWrite(ledPin, LOW);
  delay(2000);
  digitalWrite(ledPin, HIGH);
  for(int i = 0; i<255; i++) {
    analogWrite(pompePin, i);
    Serial.println(i);
    delay(50);
  }
  for(int i = 255; i>0; i--) {
    analogWrite(pompePin, i);
    Serial.println(i);
    delay(50);
  }
  digitalWrite(pompePin, LOW);
  digitalWrite(ledPin, LOW);
  delay(2000);
}

void testStepper() {

  stepper1.enableOutputs();
  stepper1.setSpeed(3200);
  stepper1.runSpeed();
  digitalWrite(ledPin, HIGH);

}

void testTreuil() {

  long newEncTreuil;
  newEncTreuil = encTreuil.read();
  Input = encTreuil.read();
  myPID.Compute();

  if (newEncTreuil < Setpoint - 2) {
    digitalWrite(PHTreuil, HIGH);
    analogWrite(ENTreuil, Output);
  }
  else if (newEncTreuil > Setpoint + 2) {
    digitalWrite(PHTreuil, LOW);
    analogWrite(ENTreuil, Output);
  }
  else {
    analogWrite(ENTreuil, 0);
  }

  if (newEncTreuil != positionEncTreuil) {
    positionEncTreuil = newEncTreuil;
  }

  Serial.print("Encoder = ");
  Serial.print(newEncTreuil);
  Serial.print(" / Output = ");
  Serial.print(Output);
  Serial.println();

}

void testWaterFlow() {
  currentMicros = micros(); //on enregistre le nombre de microsecondes depuis le début du programme
  interval = currentMicros - previousMicros; //interval entre deux pulses (voir interrupt)
  freq = 1.0/(float(interval)/1000000.0); //On calcule la fréquence
  flow = freq/23.0; //On calcule le débit à partir de la formule fournie par la datasheet du débitmètre

  Serial.print("currentMicros: "); //On affiche les différentes valeurs
  Serial.print(currentMicros);
  Serial.print("/ interval: ");
  Serial.print(interval);
  Serial.print(" pulse: ");
  Serial.print(x);
  Serial.print("/ frequency: ");
  Serial.print(freq);
  Serial.print("/ flow: ");
  Serial.println(flow);
}

void testHallSensor() {
  lsPlateauState = digitalRead(lsPlateauPin);
  lsTreuilState = digitalRead(lsTreuilPin);

  Serial.print("lsPlateauState: ");
  Serial.print(lsPlateauState);
  Serial.print(" / lsTreuilState: ");
  Serial.println(lsTreuilState);
}

//ISR pour le débimètre:
void pulse() {
  x++; //On incrémente le nombre de pulses
  previousMicros = currentMicros; //On met a jour le temps depuis la dernière pulse
  //digitalWrite(ledPin, HIGH);
}