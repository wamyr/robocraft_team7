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
  digitalWrite(dir_motor_1, LOW);
  digitalWrite(dir_motor_2, LOW);
  int speed = 250;
  analogWrite(PWM_pin_1, speed);
  analogWrite(PWM_pin_2, speed);
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
