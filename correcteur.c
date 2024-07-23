#include <Encoder.h>

// Définir les broches
const int PLA = 2;        // Broche A de l'encodeur principal
const int PRA = 7;       
const int PLB = 3;        // Broche A de l'encodeur secondaire
const int PRB = 8;        
const int MLpos = 5;      // PWM pour la vitesse positive du moteur gauche
const int MLneg = 6;      // PWM pour la vitesse négative du moteur gauche
const int MRpos = 10;     // PWM pour la vitesse positive du moteur droit
const int MRneg = 11;     // PWM pour la vitesse négative du moteur droit
const int potPin = A1;    // Potentiomètre pour régler la vitesse souhaitée

// Initialiser les objets Encoder
Encoder myEnc(PLA, PRA);
Encoder myEnc1(PLB, PRB);

// Variables globales
volatile long encoderCount = 0;
long encDiff0 = 0;
long encDiff00 = 0;
unsigned long lastUpdateTime = 0;
double desiredSpeed = 0;  // Vitesse souhaitée
double currentSpeed = 0;  // Vitesse actuelle
float r = 0.05;           // Rayon des roues
float b = 0.2;            // Distance entre les roues
float C = 100;            // Résolution de l'encodeur
float deltat = 0.1;       // Différence de temps en secondes
float wleft = 0;
float wright = 0;
float proerr[2] = {0, 0};
float deverr[2] = {0, 0};
float interr[2] = {0, 0};
float Gmot[2] = {0, 0};

// Constantes pour le PID
const float Kp = 1.0;
const float Ki = 0.0;
const float Kd = 0.0;

// Structure pour les données d'odométrie
struct odomMsg {
  float v;
  float w;
  float x;
  float y;
  float theta;
};

// Initialiser les valeurs de pose
odomMsg odom = {0, 0, 0, 0, 0};

void setup() {
  // Initialiser les broches des encodeurs
  pinMode(PLA, INPUT);
  pinMode(PRA, INPUT);
  pinMode(PLB, INPUT);
  pinMode(PRB, INPUT);

  // Initialiser les broches des moteurs
  pinMode(MLpos, OUTPUT);
  pinMode(MLneg, OUTPUT);
  pinMode(MRpos, OUTPUT);
  pinMode(MRneg, OUTPUT);

  // Initialiser la communication série
  Serial.begin(9600);

  // Attacher l'interruption
  attachInterrupt(digitalPinToInterrupt(PRA), encFunc, CHANGE);
  
  analogWrite(MLpos,200);
  analogWrite(MLneg,0);
  analogWrite(MRpos,200);
  analogWrite(MRneg,0);
}

void loop() {
  // Lire la vitesse souhaitée depuis le potentiomètre
  desiredSpeed = analogRead(potPin) / 1023.0 * 255.0;

  // Mettre à jour les données toutes les 100 ms (10 Hz)
  if (millis() - lastUpdateTime >= 100) {
    encUpdate();
    float* vel = cmd_vel();
    float wheels[2];
    cmd_vel2wheels(vel, b, r, wheels);
    pid_controller();
    poseUpdate(wheels);
    PIDError(wheels);
    lastUpdateTime = millis();
  }
}

// Fonction d'interruption pour l'encodeur
void encFunc() {
  encoderCount++;
}

// Fonction pour mettre à jour les différences d'encodeur
void encUpdate() {
  long encDiff1 = myEnc.read();
  long encDiff11 = myEnc1.read();

  int diff = encDiff1 - encDiff0;
  int difff = encDiff11 - encDiff00;

  Serial.print("La différence d'encodeur principal est de ");
  Serial.println(diff);
  Serial.print("La différence d'encodeur secondaire est de ");
  Serial.println(difff);

  encDiff0 = encDiff1;
  encDiff00 = encDiff11;
}

// Fonction de commande de vitesse
float* cmd_vel() {
  static float vel[2] = {0.05, 1}; // Exemple de vitesses
  return vel;
}

// Fonction pour convertir les vitesses de commande en vitesses des roues
void cmd_vel2wheels(float vel[2], float axis_length, float radius, float wheels[2]) {
  wheels[0] = (vel[0] - (axis_length / 2) * vel[1]) / radius; // gauche
  wheels[1] = (vel[0] + (axis_length / 2) * vel[1]) / radius; // droite
  Serial.print("Vitesses des roues : Gauche = ");
  Serial.print(wheels[0]);
  Serial.print(", Droite = ");
  Serial.println(wheels[1]);
}

// Fonction pour mettre à jour les données d'odométrie
void poseUpdate(float wheels[2]) {
  long encDiff1 = myEnc.read();
  long encDiff11 = myEnc1.read();

  odom.v = ((2.0 * PI * r) / C) * ((encDiff1 + encDiff11) / 2.0) / deltat;
  odom.w = ((2.0 * PI * r) / C) * ((encDiff1 - encDiff11) / b) / deltat;
  odom.theta = atan2(sin(odom.theta + odom.w * deltat), cos(odom.theta + odom.w * deltat));
  odom.x = odom.x + odom.v * cos(odom.theta) * deltat;
  odom.y = odom.y + odom.v * sin(odom.theta) * deltat;

  wleft =(odom.v-(b/2*odom.w))/r ;
  wright =(odom.v+(b/2*odom.w))/r ;

  Serial.print("Pose: x = ");
  Serial.print(odom.x);
  Serial.print(", y = ");
  Serial.print(odom.y);
  Serial.print(", theta = ");
  Serial.println(odom.theta);
}

// Fonction de calcul des erreurs PID
void PIDError(float wheels[2]) {
  deverr[0] = proerr[0] - wheels[0]; // ed(i+1) = ep(i+1) - ep(i)
  deverr[1] = proerr[1] - wheels[1];

  proerr[0] = wheels[0] - wleft; // ep(i+1)
  proerr[1] = wheels[1] - wright;

  interr[0] += proerr[0]; // el(i+1) = el(i) + ep(i+1)
  interr[1] += proerr[1];

  Serial.print("Proportional Error Gauche = ");
  Serial.print(proerr[0]);
  Serial.print(", Droite = ");
  Serial.println(proerr[1]);

  Serial.print("Derivative Error Gauche = ");
  Serial.print(deverr[0]);
  Serial.print(", Droite = ");
  Serial.println(deverr[1]);

  Serial.print("Integral Error Gauche = ");
  Serial.print(interr[0]);
  Serial.print(", Droite = ");
  Serial.println(interr[1]);
}

void pid_controller() {
  Gmot[0] = Kp * proerr[0] + Ki * interr[0] * deltat + Kd * deverr[0] / deltat;
  Gmot[1] = Kp * proerr[1] + Ki * interr[1] * deltat + Kd * deverr[1] / deltat;

  // Commande moteur gauche
  if (Gmot[0] > 0.0) {
    analogWrite(MLpos, (int)Gmot[0]);
    analogWrite(MLneg, 0);
  } else {
    analogWrite(MLpos, 0);
    analogWrite(MLneg, (int)fabs(Gmot[0]));
  }

  // Commande moteur droit
  if (Gmot[1] > 0.0) {
    analogWrite(MRpos, (int)Gmot[1]);
    analogWrite(MRneg, 0);
  } else {
    analogWrite(MRpos, 0);
    analogWrite(MRneg, (int)fabs(Gmot[1]));
  }

  Serial.print("Commande moteur Gauche = ");
  Serial.print(Gmot[0]);
  Serial.print(", Droite = ");
  Serial.println(Gmot[1]);
}
