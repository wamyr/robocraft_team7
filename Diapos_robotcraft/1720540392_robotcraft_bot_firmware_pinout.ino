// define motors pins
#define M1R_DIR 5	// RIGHT motor direction | Set pin to LOW to move front ! HIGH to move back
#define M1R_PWM 4	// Set 0 to 255 to set RIGHT motor speed
#define M2L_DIR 6	// LEFT motor direction  | Set pin to LOW to move front ! HIGH to move back
#define M2L_PWM 9	// Set 0 to 255 to set LEFT motor speed

// define range sensors pins (IR sensors)
#define IR_FRONT A2	// IF1
#define IR_RIGHT A3	// IF2
#define IR_LEFT  A4	// IF3

// Encoders Pins
#define ENC_M1R_A 18	// RIGHT encoder channel A - C1
#define ENC_M1R_B 19	// RIGHT encoder channel B - C2
#define ENC_M2L_A 3	// LEFT encoder channel A - C1
#define ENC_M2L_B 2	// LEFT encoder channel B - C2

// SMART LED Pin
#define SMART_LED_P 10
#define NUMPIXELS   2

