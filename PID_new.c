#include <Encoder.h>

// Définir les broches
const int PLA = 2;        // Broche A de l'encodeur principal
const int PRA = 18;       
const int PLB = 3;        // Broche A de l'encodeur secondaire
const int PRB = 19;        
#define dir_motor_1 5
#define dir_motor_2 6
#define PWM_pin_1 4
#define PWM_pin_2 9

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
float r = 0.016;          // Rayon des roues
float b = 0.1;            // Distance entre les roues
float C = 8400;           // Résolution de l'encodeur (8400 apparement à tester sans doute, 596 sur random datasheet)
float deltat = 0.1;       // Différence de temps en secondes
float wleft = 0;
float wright = 0;
float proerr[2] = {0, 0};
float deverr[2] = {0, 0};
float interr[2] = {0, 0};
float Gmot[2] = {0, 0};

// Constantes pour le PID
const float Kp = 0.2;
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
  pinMode(PWM_pin_1, OUTPUT);
  pinMode(PWM_pin_2, OUTPUT);
  pinMode(dir_motor_1, OUTPUT);
  pinMode(dir_motor_1, OUTPUT);

  // Initialiser la communication série
  Serial.begin(9600);

  // Attacher l'interruption
  attachInterrupt(digitalPinToInterrupt(PRA), encFunc, CHANGE);
  
  digitalWrite(dir_motor_1, 0);
  analogWrite(PWM_pin_1, 200);
  analogWrite(PWM_pin_2, 200);
  digitalWrite(dir_motor_2, 0);
  
}

void loop() {
  // Mettre à jour les données toutes les 100 ms (10 Hz)
  if (millis() - lastUpdateTime >= 100) {
    encUpdate();
    float* vel = cmd_vel();
    float wheels[2];
    cmd_vel2wheels(vel, b, r, wheels);
    poseUpdate(); //wheels ?
    PIDError(wheels);
    pid_controller();
    lastUpdateTime = millis();

    // Envoyer la vitesse du moteur droit au port série pour le tracé
    Serial.print("Temps = ");
    Serial.print(millis() / 1000.0);
    Serial.print("odom.v = ");
    Serial.println(odom.v);
    Serial.print(", odom.w = ");
    Serial.println(odom.w);
    
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

  //Serial.print("La différence d'encodeur principal est de ");
  //Serial.println(diff);
  //Serial.print("La différence d'encodeur secondaire est de ");
  //Serial.println(difff);

  encDiff0 = encDiff1;
  encDiff00 = encDiff11;
}

// Fonction de commande de vitesse
float* cmd_vel() {
  static float vel[2] = {0.05, 0}; // Exemple de vitesses
  return vel;
}

// Fonction pour convertir les vitesses de commande en vitesses des roues
void cmd_vel2wheels(float vel[2], float axis_length, float radius, float wheels[2]) {
  wheels[0] = (vel[0] - (axis_length / 2) * vel[1]) / radius; // gauche
  wheels[1] = (vel[0] + (axis_length / 2) * vel[1]) / radius; // droite
  // Commenté pour le tracé
  // Serial.print("Vitesses des roues : Gauche = ");
  // Serial.print(wheels[0]);
  // Serial.print(", Droite = ");
  // Serial.println(wheels[1]);
}

// Fonction pour mettre à jour les données d'odométrie
void poseUpdate() { //float wheels[2] ?
  long encDiff1 = myEnc.read();
  long encDiff11 = myEnc1.read();

  odom.v = ((2.0 * PI * r) / C) * ((encDiff1 + encDiff11) / 2.0) / deltat;
  odom.w = ((2.0 * PI * r) / C) * ((encDiff1 - encDiff11) / b) / deltat;
  odom.theta = atan2(sin(odom.theta + odom.w * deltat), cos(odom.theta + odom.w * deltat));
  odom.x = odom.x + odom.v * cos(odom.theta) * deltat;
  odom.y = odom.y + odom.v * sin(odom.theta) * deltat;

  wleft = (odom.v - (b / 2 * odom.w)) / r;
  wright = (odom.v + (b / 2 * odom.w)) / r;

  
  Serial.print("Pose: x = ");
  Serial.print(odom.x);
  Serial.print(", y = ");
  Serial.print(odom.y);
  Serial.print(", theta = ");
  Serial.println(odom.theta);
}

// Fonction de calcul des erreurs PID
void PIDError(float wheels[2]) {
  deverr[0] = -proerr[0]; // ed(i+1) = ep(i+1) - ep(i)
  deverr[1] = -proerr[1];

  proerr[0] = wheels[0] - wleft; // ep(i+1)
  proerr[1] = wheels[1] - wright;

  deverr[0] += proerr[0]; // ed(i+1) = ep(i+1) - ep(i)
  deverr[1] += proerr[1];

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


  //En théorie 0 sur la dir met les deux vers l'avant
  // Commande moteur gauche ==> Pin 2
  if (Gmot[0] > 0.0) {
    analogWrite(PWM_pin_2, (int)Gmot[0]);
    digitalWrite(dir_motor_2, 0);
  } else {
    digitalWrite(dir_motor_2, 1);
    analogWrite(PWM_pin_2, (int)fabs(Gmot[0]));
  }

  // Commande moteur droit ==> Pin 1
  if (Gmot[1] > 0.0) {
    analogWrite(PWM_pin_1, (int)Gmot[1]);
    digitalWrite(dir_motor_1, 0);
  } else {
    digitalWrite(dir_motor_1, 1);
    analogWrite(PWM_pin_1, (int)fabs(Gmot[1]));
  }

  // Commenté pour le tracé
  Serial.print("Commande moteur Gauche = ");
  Serial.print(Gmot[0]);
  Serial.print(", Droite = ");
  Serial.println(Gmot[1]);
}