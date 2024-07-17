#include "SharpIR.h"

#define IRPinFront A2
#define IRPinRight A3
#define IRPinLeft A4
#define model 1080

int distance_mmFront;
int distance_mmRight;
int distance_mmLeft;

SharpIR sensorFront = SharpIR(IRPinFront, model);
SharpIR sensorRight = SharpIR(IRPinRight, model);
SharpIR sensorLeft = SharpIR(IRPinLeft, model);

void setup() {
  Serial.begin(9600);
}

void loop() {
  sensF();
  sensR();
  sensL();
  delay(100);
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
