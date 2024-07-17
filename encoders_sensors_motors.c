#include <Encoder.h>
#include "SharpIR.h"

// Définir les broches des moteurs
#define dir_motor_1 5
#define dir_motor_2 6
#define PWM_pin_1 4
#define PWM_pin_2 9

// Définir les broches des encodeurs
const int encoderPinA1 = 3;   // Broche A de l'encodeur du moteur 1 (D3)
const int encoderPinB1 = 2;   // Broche B de l'encodeur du moteur 1 (D2)
const int encoderPinA2 = 18;  // Broche A de l'encodeur du moteur 2 (D18)
const int encoderPinB2 = 19;  // Broche B de l'encodeur du moteur 2 (D19)

// Définir les broches des capteurs IR
#define IRPinFront A2
#define IRPinRight A3
#define IRPinLeft A4
#define model 1080

// Variables pour stocker les positions des encodeurs
long previousPositionL = 0;
long previousPositionR = 0;
unsigned long lastUpdateTime = 0;

// Variables pour stocker les distances des capteurs IR
int distance_mmFront;
int distance_mmRight;
int distance_mmLeft;

// Instances des classes Encoder et SharpIR
Encoder encL(encoderPinA1, encoderPinB1);
Encoder encR(encoderPinA2, encoderPinB2);
SharpIR sensorFront = SharpIR(IRPinFront, model);
SharpIR sensorRight = SharpIR(IRPinRight, model);
SharpIR sensorLeft = SharpIR(IRPinLeft, model);

void setup() {
  Serial.begin(9600);

  // Initialiser les broches des moteurs
  pinMode(dir_motor_1, OUTPUT);
  pinMode(dir_motor_2, OUTPUT);
  pinMode(PWM_pin_1, OUTPUT);
  pinMode(PWM_pin_2, OUTPUT);
}

void loop() {
  // Lire les distances des capteurs IR
  sensF();
  sensR();
  sensL();
  
  // Lire les différences des encodeurs toutes les 100ms (10Hz)
  if (millis() - lastUpdateTime >= 100) {
    readEncoderDifferences();
    lastUpdateTime = millis();
  }

  // Contrôler les moteurs
  digitalWrite(dir_motor_1, LOW);
  digitalWrite(dir_motor_2, LOW);
  int speed = 250;
  analogWrite(PWM_pin_1, speed);
  analogWrite(PWM_pin_2, speed);
  
  delay(10);
}

void sensF() {
  int distance_cmFront = sensorFront.distance();
  distance_mmFront = distance_cmFront * 10;
  Serial.print("Distance devant : ");
  Serial.print(distance_mmFront);
  Serial.println(" mm");
}

void sensR() {
  int distance_cmRight = sensorRight.distance();
  distance_mmRight = distance_cmRight * 10;
  Serial.print("Distance à droite : ");
  Serial.print(distance_mmRight);
  Serial.println(" mm");
}

void sensL() {
  int distance_cmLeft = sensorLeft.distance();
  distance_mmLeft = distance_cmLeft * 10;
  Serial.print("Distance à gauche : ");
  Serial.print(distance_mmLeft);
  Serial.println(" mm");
}

void readEncoderDifferences() {
  long currentPositionL = encL.read();
  long currentPositionR = encR.read();

  long differenceL = currentPositionL - previousPositionL;
  long differenceR = currentPositionR - previousPositionR;

  Serial.print("Différence moteur gauche (L) depuis la dernière requête: ");
  Serial.println(differenceL);
  Serial.print("Différence moteur droite (R) depuis la dernière requête: ");
  Serial.println(differenceR);

  // Mettre à jour les positions précédentes
  previousPositionL = currentPositionL;
  previousPositionR = currentPositionR;
}
