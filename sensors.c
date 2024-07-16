int i;
int val;
int redpin=0;
void setup()
{
  pinMode(redpin,OUTPUT);
  Serial.begin(9600);
}
void loop()
{
  i=analogRead(redpin);
  val=(6762/(i-9))-4;
  Serial.println(val);
}

// test annexe 

const int sensorPin = A4; // Définir le capteur sur l'entrée analogique A4
const int sensorMinValue = 10; // Valeur minimale pour éviter la division par zéro
const int readDelay = 1000; // Délai entre les lectures en millisecondes
const int numReadings = 10; // Nombre de lectures pour la moyenne

void setup() {
  pinMode(sensorPin, INPUT); // Définir le capteur comme une entrée
  Serial.begin(9600); // Initialiser la communication série
}

void loop() {
  int total = 0;
  int averageReading;

  // Prendre plusieurs lectures pour une moyenne
  for (int i = 0; i < numReadings; i++) {
    total += analogRead(sensorPin);
    delay(10); // Petit délai entre les lectures
  }
  
  averageReading = total / numReadings;

  // Vérifiez que la valeur lue est supérieure à la valeur minimale
  if (averageReading >= sensorMinValue) {
    int distance = (6762 / (averageReading - 9)) - 4;
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
  } else {
    Serial.println("Valeur du capteur trop basse");
  }

  delay(readDelay); // Délai avant la prochaine lecture
}
//fin

//nouveau !!!!!!


// Sharp IR GP2Y0A41SK0F Distance Test
// http://tinkcore.com/sharp-ir-gp2y0a41-skf/

#define sensor A0 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)

void setup() {
  Serial.begin(9600); // start the serial port
}

void loop() {
  
  // 5v
  float volts = analogRead(sensor)*0.0048828125;  // value from sensor * (5/1024)
  int distance = 13*pow(volts, -1); // worked out from datasheet graph
  delay(1000); // slow down serial port 
  
  if (distance <= 30){
    Serial.println(distance);   // print the distance
  }
}
