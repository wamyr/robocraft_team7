https://www.makerguides.com/sharp-gp2y0a21yk0f-ir-distance-sensor-arduino-tutorial/

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

/* SHARP GP2Y0A21YK0F IR distance sensor with 
   Arduino and SharpIR library example code. 
   More info: https://www.makerguides.com */

// Include the library:
#include "SharpIR.h"

// Define model and input pin:
#define IRPin A0
#define model 1080

// Create variable to store the distance:
int distance_cm;

/* Model :
   GP2Y0A02YK0F --> 20150
   GP2Y0A21YK0F --> 1080
   GP2Y0A710K0F --> 100500
   GP2YA41SK0F  --> 430
*/

// Create a new instance of the SharpIR class:
SharpIR mySensor = SharpIR(IRPin, model);

void setup() {
  // Begin serial communication at a baudrate of 9600:
  Serial.begin(9600);
}

void loop() {
  // Number of readings to take for averaging
  const int numReadings = 10;
  int totalDistance = 0;

  for (int i = 0; i < numReadings; i++) {
    int reading = mySensor.distance();
    if (reading > 0) { // Check if the reading is valid
      totalDistance += reading;
    } else {
      Serial.println("Invalid reading");
    }
    delay(50); // Short delay between readings
  }

  // Calculate the average distance
  distance_cm = totalDistance / numReadings;

  // Print the measured distance to the serial monitor:
  Serial.print("Mean distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  delay(1000); // Delay before the next measurement
}

//fin
//nouveau 

#include "SharpIR.h"

// Définir le modèle et la broche d'entrée :
#define IRPin A4
#define model 1080

// Créer une variable pour stocker la distance :
int distance_cm;

/* Modèles pris en charge :
   GP2Y0A02YK0F --> 20150
   GP2Y0A21YK0F --> 1080
   GP2Y0A710K0F --> 100500
   GP2YA41SK0F  --> 430
*/

// Créer une nouvelle instance de la classe SharpIR :
SharpIR mySensor = SharpIR(IRPin, model);

void setup() {
  // Initialiser la communication série à 9600 baud :
  Serial.begin(9600);
}

void loop() {
  // Obtenir une mesure de distance et la stocker dans distance_cm :
  distance_cm = mySensor.distance();

  // Afficher la distance mesurée sur le moniteur série :
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  // Délai avant la prochaine mesure :
  delay(100);
}
