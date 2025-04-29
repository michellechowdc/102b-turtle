#include <ESP32Servo.h>
/* FRONT FLIPPER, reset to starting position */

Servo R_roll_servo; // object representing actual servo
Servo R_yaw_servo;
Servo R_pitch_servo;
const int R_ROLL_PIN = 33; // base servo, connected to body
const int R_YAW_PIN = 27; // middle servo
const int R_PITCH_PIN = 12; // end servo, connected to flipper

Servo L_roll_servo;
Servo L_yaw_servo;
Servo L_pitch_servo;
const int L_ROLL_PIN = 14; // base servo, connected to body
const int L_YAW_PIN = 32; // middle servo
const int L_PITCH_PIN = 15; // end servo, connected to flipper

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

void loop() {
  // put your main code here, to run repeatedly:
  // roll_servo.write(0); // move MG996R's shaft to angle 0°
  // delay(1000); // wait for one second
  // roll_servo.write(45); // move MG996R's shaft to angle 45°
  // delay(1000); // wait for one second 
  // roll_servo.write(90); // move MG996R's shaft to angle 90°
  // delay(1000); // wait for one second
  // roll_servo.write(135); // move MG996R's  shaft to angle 135°
  // delay(1000); // wait for one second
  // roll_servo.write(180); // move MG996R's shaft to angle 180°
  // delay(1000); // wait for one second

  // yaw_servo.write(0); // move MG996R's shaft to angle 0°
  // delay(1000); // wait for one second
  // yaw_servo.write(45); // move MG996R's shaft to angle 45°
  // delay(1000); // wait for one second 
  // yaw_servo.write(90); // move MG996R's shaft to angle 90°
  // delay(1000); // wait for one second
  // yaw_servo.write(135); // move MG996R's shaft to angle 135°
  // delay(1000); // wait for one second
  // yaw_servo.write(180); // move MG996R's shaft to angle 180°
  // delay(1000); // wait for one second

  // pitch_servo.write(0); // move MG996R's shaft to angle 0°
  // delay(1000); // wait for one second
  // pitch_servo.write(45); // move MG996R's shaft to angle 45°
  // delay(1000); // wait for one second 
  // pitch_servo.write(90); // move MG996R's shaft to angle 90°
  // delay(1000); // wait for one second
  // pitch_servo.write(135); // move MG996R's shaft to angle 135°
  // delay(1000); // wait for one second
  // pitch_servo.write(180); // move MG996R's shaft to angle 180°
  // delay(1000); // wait for one second
}
