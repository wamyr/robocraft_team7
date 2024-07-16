// Définition des broches de l'encodeur
const int pinA = 3;  // Broche A de l'encodeur connectée à la broche D3 de l'Arduino
const int pinB = 2;  // Broche B de l'encodeur connectée à la broche D2 de l'Arduino

// Variables pour la gestion de l'encodeur
volatile long position = 0;  // Position actuelle de l'encodeur

void setup() {
  // Initialisation des broches de l'encodeur en entrée avec pull-up
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  // Interruption sur changement d'état pour les broches de l'encodeur
  attachInterrupt(digitalPinToInterrupt(pinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), updateEncoder, CHANGE);

  // Début de la communication série
  Serial.begin(9600);
}

void loop() {
  // Affichage de la position de l'encodeur
  Serial.println(position);

  // Ajoutez ici d'autres instructions pour contrôler votre moteur en fonction de la position
  // Par exemple, utiliser la position pour contrôler un servomoteur ou un moteur pas à pas

  delay(100);  // Délai pour ralentir la boucle
}

// Fonction de mise à jour de l'encodeur (appelée lors d'un changement d'état)
void updateEncoder() {
  // Lecture des états actuels des broches A et B de l'encodeur
  int stateA = digitalRead(pinA);
  int stateB = digitalRead(pinB);

  // Calcul de la direction de rotation
  if (stateA == stateB) {
    position++;
  } else {
    position--;
  }
}
