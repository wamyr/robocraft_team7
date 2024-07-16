#include <Encoder.h>

// Define pin assignments for encoders
#define encoderLPin1 2 // Example: External interrupt pin for encoder L (left wheel)
#define encoderLPin2 3 // Example: Additional pin for encoder L
#define encoderRPin1 7 // Example: External interrupt pin for encoder R (right wheel)
#define encoderRPin2 8 // Example: Additional pin for encoder R

// Create instances of the Encoder class for each wheel
Encoder encL(encoderLPin1, encoderLPin2);
Encoder encR(encoderRPin1, encoderRPin2);

// Variables to store pulse counts
long pulseCountL = 0;
long pulseCountR = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderLPin1), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRPin1), updateEncoderR, CHANGE);
}

void loop() {
  // Read pulse counts and reset variables
  long pulsesL = pulseCountL;
  long pulsesR = pulseCountR;
  pulseCountL = 0; // Reset pulse count for next measurement
  pulseCountR = 0; // Reset pulse count for next measurement

  // Print pulse counts to serial monitor
  Serial.print("Pulses counted by encoder L: ");
  Serial.println(pulsesL);
  Serial.print("Pulses counted by encoder R: ");
  Serial.println(pulsesR);

  delay(1000); // Adjust delay as needed for your application
}

// Interrupt service routines for encoder pulses
void updateEncoderL() {
  pulseCountL++;
}

void updateEncoderR() {
  pulseCountR++;
}
