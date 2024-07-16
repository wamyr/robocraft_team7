#include <Encoder.h>

// Définition des broches pour les encodeurs
#define encoderLPinA 2 // Broche A de l'encodeur de gauche
#define encoderLPinB 3 // Broche B de l'encodeur de gauche
#define encoderRPinA 4 // Broche A de l'encodeur de droite
#define encoderRPinB 5 // Broche B de l'encodeur de droite

// Création d'instances de la classe Encoder
Encoder encL(encoderLPinA, encoderLPinB);
Encoder encR(encoderRPinA, encoderRPinB);

void setup() {
  Serial.begin(9600);
  // Attach interrupt service routines if needed
}

void loop() {
  // Lecture des compteurs d'encodeur
  long countL = encL.read();
  long countR = encR.read();

  // Affichage des valeurs lues sur le moniteur série
  Serial.print("Encoder L count: ");
  Serial.println(countL);
  Serial.print("Encoder R count: ");
  Serial.println(countR);

  delay(100); // Ajoutez un délai si nécessaire pour éviter de lire trop souvent
}
