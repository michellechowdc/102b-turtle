#include <ESP32Servo.h>
/* FRONT FLIPPER, reset to starting position */
Servo R_roll_servo; // object representing actual servo
Servo R_yaw_servo;
const int R_ROLL_PIN = 14; // base servo, connected to body
const int R_YAW_PIN = 32; // connected to flipper

Servo L_roll_servo;
Servo L_yaw_servo;
const int L_ROLL_PIN = 25; // base servo, connected to body
const int L_YAW_PIN = 26; // middle servo

/* BACK FLIPPERS */
Servo L_back_servo; // object representing actual servo
Servo R_back_servo;
const int R_BACK_PIN = 4; // base servo, connected to body
const int L_BACK_PIN = 5; // connected to flipper

void setup() {
  Serial.begin(115200);

  R_roll_servo.attach(R_ROLL_PIN); // connected to GPIO pin
  R_yaw_servo.attach(R_YAW_PIN);
  R_roll_servo.write(0); // reset position
  R_yaw_servo.write(0);

  L_roll_servo.attach(L_ROLL_PIN); // connected to GPIO pin
  L_yaw_servo.attach(L_YAW_PIN);
  L_roll_servo.write(0); // reset position
  L_yaw_servo.write(0);

  L_back_servo.attach(L_BACK_PIN);
  R_back_servo.attach(R_BACK_PIN);
  L_back_servo.write(0);
  R_back_servo.write(0);
}

void loop() {
  // put your main code here, to run repeatedly:
  R_roll_servo.write(0); // move MG996R's shaft to angle 0°
  // R_yaw_servo.write(0);
  L_roll_servo.write(0);
  // L_yaw_servo.write(0);
  // L_back_servo.write(0);
  // R_back_servo.write(0);
  delay(1000); // wait for one second
  R_roll_servo.write(45); // move MG996R's shaft to angle 45°
  // R_yaw_servo.write(45);
  L_roll_servo.write(45);
  // L_yaw_servo.write(45);
  // L_back_servo.write(45);
  // R_back_servo.write(45);
  delay(1000); // wait for one second 
  R_roll_servo.write(90); // move MG996R's shaft to angle 90°
  // R_yaw_servo.write(90);
  L_roll_servo.write(90);
  // L_yaw_servo.write(90);
  // L_back_servo.write(90);
  // R_back_servo.write(90);
  delay(1000); // wait for one second
  // R_roll_servo.write(135); // move MG996R's  shaft to angle 135°
  // R_yaw_servo.write(135);
  // L_roll_servo.write(135);
  // L_yaw_servo.write(135);
  // L_back_servo.write(135);
  // R_back_servo.write(135);
  delay(1000); // wait for one second
  // R_roll_servo.write(180); // move MG996R's shaft to angle 180°
  // R_yaw_servo.write(180);
  // L_roll_servo.write(180);
  // L_yaw_servo.write(180);
  // L_back_servo.write(180);
  // R_back_servo.write(180);
  delay(1000); // wait for one second
}
