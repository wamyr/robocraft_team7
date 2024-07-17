// Moteur 1 - Définition des broches de l'encodeur
const int encoderPinA1 = 3;   // Broche A de l'encodeur du moteur 1 (D3)
const int encoderPinB1 = 2;   // Broche B de l'encodeur du moteur 1 (D2)

// Moteur 2 - Définition des broches de l'encodeur
const int encoderPinA2 = 18;  // Broche A de l'encodeur du moteur 2 (D18)
const int encoderPinB2 = 19;  // Broche B de l'encodeur du moteur 2 (D19)

// Variables pour la gestion des encodeurs des deux moteurs
volatile long position1 = 0;  // Position actuelle de l'encodeur du moteur 1
volatile long position2 = 0;  // Position actuelle de l'encodeur du moteur 2

void setup() {
  // Initialisation des broches des encodeurs en entrée avec pull-up
  pinMode(encoderPinA1, INPUT_PULLUP);
  pinMode(encoderPinB1, INPUT_PULLUP);
  pinMode(encoderPinA2, INPUT_PULLUP);
  pinMode(encoderPinB2, INPUT_PULLUP);

  // Interruptions sur changement d'état pour les encodeurs
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB2), updateEncoder2, CHANGE);

  // Début de la communication série
  Serial.begin(9600);
}

void loop() {
  // Affichage de la position des encodeurs des deux moteurs
  Serial.print("Position moteur 1: ");
  Serial.println(position1);
  Serial.print("Position moteur 2: ");
  Serial.println(position2);

  // Ajoutez ici d'autres instructions pour contrôler vos moteurs en fonction des positions

  delay(100);  // Délai pour ralentir la boucle
}

// Fonction de mise à jour de l'encodeur du moteur 1 (appelée lors d'un changement d'état)
void updateEncoder1() {
  int stateA = digitalRead(encoderPinA1);
  int stateB = digitalRead(encoderPinB1);
  if (stateA == stateB) {
    position1++;
  } else {
    position1--;
  }
}

// Fonction de mise à jour de l'encodeur du moteur 2 (appelée lors d'un changement d'état)
void updateEncoder2() {
  int stateA = digitalRead(encoderPinA2);
  int stateB = digitalRead(encoderPinB2);
  if (stateA == stateB) {
    position2++;
  } else {
    position2--;
  }
}

// test nouvelle fonction !!!!!!!!!!!!!

#include <Encoder.h>

// Moteur 1 - Définition des broches de l'encodeur
const int encoderPinA1 = 3;   // Broche A de l'encodeur du moteur 1 (D3)
const int encoderPinB1 = 2;   // Broche B de l'encodeur du moteur 1 (D2)

// Moteur 2 - Définition des broches de l'encodeur
const int encoderPinA2 = 18;  // Broche A de l'encodeur du moteur 2 (D18)
const int encoderPinB2 = 19;  // Broche B de l'encodeur du moteur 2 (D19)

// Création des instances Encoder pour chaque moteur
Encoder encL(encoderPinA1, encoderPinB1);
Encoder encR(encoderPinA2, encoderPinB2);

// Variables pour stocker les positions précédentes
long previousPositionL = 0;
long previousPositionR = 0;
unsigned long lastUpdateTime = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (millis() - lastUpdateTime >= 100) {  // 10Hz frequency
    readEncoderDifferences();
    lastUpdateTime = millis();
  }

  // Ajoutez ici d'autres instructions pour contrôler vos moteurs en fonction des positions
  delay(10);  // Délai pour ralentir la boucle
}

// Fonction pour lire les valeurs absolues des encodeurs
void readEncoderValues() {
  long positionL = encL.read();
  long positionR = encR.read();

  Serial.print("Position moteur gauche (L): ");
  Serial.println(positionL);
  Serial.print("Position moteur droite (R): ");
  Serial.println(positionR);
}

// Fonction pour lire les différences des encodeurs depuis la dernière requête
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
