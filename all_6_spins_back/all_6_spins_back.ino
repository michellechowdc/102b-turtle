#include <ESP32Servo.h>
/* BACK FLIPPER, reset to starting position */

Servo R_roll_servo; // object representing actual servo
Servo R_yaw_servo;
Servo R_pitch_servo;
const int R_ROLL_PIN = 26; // base servo, connected to body
const int R_YAW_PIN = 25; // middle servo
const int R_PITCH_PIN = 34; // end servo, connected to flipper

Servo L_roll_servo;
Servo L_yaw_servo;
Servo L_pitch_servo;
const int L_ROLL_PIN = 39; // base servo, connected to body
const int L_YAW_PIN = 36; // middle servo
const int L_PITCH_PIN = 4; // end servo, connected to flipper

void setup() {
  Serial.begin(115200);

  R_roll_servo.attach(R_ROLL_PIN); // connected to GPIO pin
  R_yaw_servo.attach(R_YAW_PIN);
  R_pitch_servo.attach(R_PITCH_PIN);
  R_roll_servo.write(0); // reset position
  R_yaw_servo.write(0);
  R_pitch_servo.write(0);  

  L_roll_servo.attach(L_ROLL_PIN); // connected to GPIO pin
  L_yaw_servo.attach(L_YAW_PIN);
  L_pitch_servo.attach(L_PITCH_PIN);
  L_roll_servo.write(0); // reset position
  L_yaw_servo.write(0);
  L_pitch_servo.write(0);
}

int angles[5] = {0, 45, 90, 135, 180}; // degrees
void loop() {
  for (int i = 0; i < 5; i++) {
    R_roll_servo.write(angles[i]);
    delay(1000); // 1 second
  }

  for (int i = 0; i < 5; i++) {
    R_yaw_servo.write(angles[i]);
    delay(1000);
  }

  for (int i = 0; i < 5; i++) {
    R_pitch_servo.write(angles[i]);
    delay(1000);
  }

  for (int i = 0; i < 5; i++) {
    L_roll_servo.write(angles[i]);
    delay(1000); // 1 second
  }

  for (int i = 0; i < 5; i++) {
    L_yaw_servo.write(angles[i]);
    delay(1000);
  }

  for (int i = 0; i < 5; i++) {
    L_pitch_servo.write(angles[i]);
    delay(1000);
  }
}
