#define M1R_DIR 5  // RIGHT motor direction | Set pin to LOW to move front ! HIGH to move back
#define M1R_PWM 4  // Set 0 to 255 to set RIGHT motor speed
#define M2L_DIR 6  // LEFT motor direction  | Set pin to LOW to move front ! HIGH to move back
#define M2L_PWM 9  // Set 0 to 255 to set LEFT motor speed

#include <Encoder.h>

// Encoders Pins
#define ENC_M2L_A 18  // RIGHT encoder channel A - C1
#define ENC_M2L_B 19  // RIGHT encoder channel B - C2
#define ENC_M1R_A 3   // LEFT encoder channel A - C1
#define ENC_M1R_B 2   // LEFT encoder channel B - C2

Encoder myEncL(ENC_M2L_A, ENC_M2L_B);
Encoder myEncR(ENC_M1R_A, ENC_M1R_B);

float r = 0.016;
float b = 0.097;
float c = 100.0;
float pulses_rev = 8400.0;
float time = 0.1;
float wleft = 0.0;
float wright = 0.0;

struct odomMsg {
  float v;
  float w;
  float x;
  float y;
  float yaw;

  odomMsg()
    : v(0), w(0), x(0), y(0), yaw(0) {}
};

odomMsg poseUpdate(float encDiff[2]);
float* cld_vel2wheels(float vel[2]);
float* cmd_vel();
float* encUpdate();
void pid_controller(float wDes[2], float wReal[2]);

void setup() {
  Serial.begin(9600);
  pinMode(M1R_DIR, OUTPUT);
  pinMode(M1R_PWM, OUTPUT);
  pinMode(M2L_DIR, OUTPUT);
  pinMode(M2L_PWM, OUTPUT);

  // Initialize encoder pins
  pinMode(ENC_M1R_A, INPUT);
  pinMode(ENC_M1R_B, INPUT);
  pinMode(ENC_M2L_A, INPUT);
  pinMode(ENC_M2L_B, INPUT);

  // analogWrite(M1R_DIR, LOW);
  // analogWrite(M1R_PWM, 100);

  // analogWrite(M2L_DIR, LOW);
  // analogWrite(M2L_PWM, 110);
}

void loop() {
  unsigned long tnow = millis();
  long nt = 0, n = 0;
  while (1) {
    if (millis() - tnow >= time*1000.0) {
      tnow = millis();

      // Give the robot a desired linear and angular velocities
      float *encs = cmd_vel();  // gives values
      // Serial.print("(v , w) : (");
      // Serial.print(encs[0]);
      // Serial.print(" , ");
      // Serial.print(encs[1]);
      // Serial.println(")");

      // Calculate the desired angular velocities of the wheels
      // Serial.print("cld_vel2wheels: (wL , wR) : (");
      float* wheels = cld_vel2wheels(encs);
      // Serial.print(wheels[0]);
      // Serial.print(" , ");
      // Serial.print(wheels[1]);
      // Serial.println(")");
      float wD[2] = {wheels[0],wheels[1]};

      // Calculate the desired angular velocities of the wheels
      // Serial.print("encUpdate: (Nl , Nr) : (");
      float* wheelEnc = encUpdate();
      /*Serial.println("Desired:");
      Serial.println(wheelEnc[0]);
      Serial.print(" , ");
      Serial.print(wheelEnc[1]);
      Serial.println(")");*/

      // Run algorithm Calculate Pose 2
      // Serial.print("Pose 2: ");
      odomMsg msg = poseUpdate(wheelEnc);
      Serial.print("v:");
      Serial.print(msg.v);
      Serial.print(" , w:");
      Serial.print(msg.w);
      Serial.print(" , x:");
      Serial.print(msg.x);
      Serial.print(" , y:");
      Serial.print(msg.y);
      Serial.print(" , yaw:");
      Serial.println(msg.yaw);

      // Calculate the real angular velocities of the wheels
      float encsR[2];
      encsR[0] = msg.v;
      encsR[1] = msg.w;

      // Serial.print("(WLR, WRR) : (");
      float* wheelsR = cld_vel2wheels(encsR);

      /*Serial.print(" , ");
      Serial.print(wheels[1]);
      Serial.println(")");*/

      
      float wR[2] = {wheelsR[0],wheelsR[1]};

      pid_controller(wD, wR);
    }
  }
}

float* cmd_vel() {
  static float vel[2] = { 0.02, 0.0 };
  return vel;
}

float* encUpdate() {
  static float encReadings[2] = { 0.0, 0.0 };
  static float encDiff[2] = { 0.0, 0.0 };

  encDiff[0] = myEncL.read() - encReadings[0];
  encDiff[1] = myEncR.read() - encReadings[1];
  encReadings[0] = (float)myEncL.read();
  encReadings[1] = (float)myEncR.read();
  // Serial.print("Read : ");
  // Serial.print((float)myEncL.read());
  // Serial.print(" , ");
  // Serial.println((float)myEncR.read());
  return encDiff;
}

float* cld_vel2wheels(float vel[2]) {
  static float wheels[2];  // so it does not clean
  wheels[0] = (vel[0] - (b / 2.0) * vel[1]) / r;
  wheels[1] = (vel[0] + (b / 2.0) * vel[1]) / r;
  return wheels;
}

odomMsg poseUpdate(float encDiff[2]) {
  static odomMsg odom;

  // Serial.println("Desired:");
  // Serial.println(encDiff[0]);
  // Serial.print(" , ");
  // Serial.print(encDiff[1]);
  // Serial.println(")");

  odom.v = (2.0 * PI * r) * (encDiff[0] + encDiff[1]) * (1.0 / (time * pulses_rev * 2.0));
  odom.w = (2.0 * PI * r) * (encDiff[1] - encDiff[0]) * (1.0 / (time * pulses_rev * b));

  odom.yaw = atan2(sin(odom.yaw + odom.w * time), cos(odom.yaw + odom.w * time));
  odom.x = odom.x + odom.v * cos(odom.yaw) * time;
  odom.y = odom.y + odom.v * sin(odom.yaw) * time;

  wleft = (odom.v - (b / 2 * odom.w)) / r;
  wright = (odom.v + (b / 2 * odom.w)) / r;

  return odom;
}

void pid_controller(float wDes[2], float wReal[2]) {
  float torque[2];
  float Kp = 0.0, Ki = 10.0, Kd = 0.0;
  static float Ep[2] = { 0.0, 0.0 }, Ei[2] = { 0.0, 0.0 }, Ed[2] = { 0.0, 0.0 };
  float G[2];

  Ed[0] = (wDes[0] - wReal[0]) - Ep[0];
  Ed[1] = (wDes[1] - wReal[1]) - Ep[1];

  Ep[0] = wDes[0] - wReal[0];
  Ep[1] = wDes[1] - wReal[1];

  Ei[0] = Ei[0] + Ep[0];
  Ei[1] = Ei[1] + Ep[1];

  G[0] = Kp * Ep[0] + Ki * Ei[0] + Kd * Ed[0];
  G[1] = Kp * Ep[1] + Ki * Ei[1] + Kd * Ed[1];

  // G[0] *= wleft;
  // G[1] *= wright;

  // Serial.print("Ep:");
  // Serial.println(Ep[0]);
  // Serial.print("Ei:");
  // Serial.println(Ei[0]);
  // Serial.print("G:");
  // Serial.println(G[0]);

  G[0] = constrain(G[0], -255, 255);
  G[1] = constrain(G[1], -255, 255);
  
  
  //Serial.println(G[1]);

  // G[0] = 100.0;
  // G[1] = 90.0;

  if (G[1] > 0.0) {
    analogWrite(M1R_DIR, LOW);
    analogWrite(M1R_PWM, (int)G[1]);
  } else {
    analogWrite(M1R_PWM, (int)fabs(G[1]));
    analogWrite(M1R_DIR, HIGH);
  }

  if (G[0] > 0.0) {
    analogWrite(M2L_DIR, LOW);
    analogWrite(M2L_PWM, (int)G[0]);
  } else {
    analogWrite(M2L_PWM, (int)fabs(G[0]));
    analogWrite(M2L_DIR, HIGH);
  }
}
