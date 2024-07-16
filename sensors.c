#include "SharpIR.h"

// Définir les modèles et les broches d'entrée :
#define IRPinFront A2
#define IRPinRight A3
#define IRPinLeft A4
#define model 1080

// Créer des variables pour stocker les distances en millimètres :
int distance_mmFront;
int distance_mmRight;
int distance_mmLeft;

/* Modèles pris en charge :
   GP2Y0A02YK0F --> 20150
   GP2Y0A21YK0F --> 1080
   GP2Y0A710K0F --> 100500
   GP2YA41SK0F  --> 430
*/

// Créer des nouvelles instances de la classe SharpIR pour chaque capteur :
SharpIR sensorFront = SharpIR(IRPinFront, model);
SharpIR sensorRight = SharpIR(IRPinRight, model);
SharpIR sensorLeft = SharpIR(IRPinLeft, model);

void setup() {
  // Initialiser la communication série à 9600 baud :
  Serial.begin(9600);
}

void loop() {
  // Obtenir une mesure de distance en centimètres pour chaque capteur :
  int distance_cmFront = sensorFront.distance();
  int distance_cmRight = sensorRight.distance();
  int distance_cmLeft = sensorLeft.distance();

  // Convertir les distances en millimètres :
  distance_mmFront = distance_cmFront * 10;
  distance_mmRight = distance_cmRight * 10;
  distance_mmLeft = distance_cmLeft * 10;

  // Afficher les distances mesurées sur le moniteur série :
  Serial.print("Distance devant : ");
  Serial.print(distance_mmFront);
  Serial.println(" mm");

  Serial.print("Distance à droite : ");
  Serial.print(distance_mmRight);
  Serial.println(" mm");

  Serial.print("Distance à gauche : ");
  Serial.print(distance_mmLeft);
  Serial.println(" mm");

  // Délai avant la prochaine mesure :
  delay(100);
}
