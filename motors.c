#define dir_motor_1 5
#define dir_motor_2 6
#define PWM_pin_1 4
#define PWM_pin_2 9

void setup(){}
void loop() {
  // Accelerate forward
  digitalWrite(dir_motor_1, LOW);
  digitalWrite(dir_motor_2, LOW);
  int speed = 250;
  analogWrite(PWM_pin_1, speed);
  analogWrite(PWM_pin_2, speed);
  delay(50);
  
  delay(2000);  // Wait for 2 seconds before repeating
}

