#include <Encoder.h>
#define dir_motor_1 5
#define dir_motor_2 6
#define PWM_pin_1 4
#define PWM_pin_2 9

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
  // Accelerate forward
  digitalWrite(dir_motor_1, LOW);
  digitalWrite(dir_motor_2, LOW);
  int speed = 250;
  analogWrite(PWM_pin_1, speed);
  analogWrite(PWM_pin_2, speed);
  
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
