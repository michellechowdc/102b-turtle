#include <ESP32Servo.h>
Servo roll_servo; // object representing actual servo
Servo yaw_servo;
Servo pitch_servo;

void setup() {
  // put your setup code here, to run once:
  roll_servo.attach(4); // connected to GPIO pin 4
  roll_servo.write(90); // reset position

  yaw_servo.attach(5);
  yaw_servo.write(90);

  pitch_servo.attach(18);
  pitch_servo.write(90);
}

void loop() {
float A = 10.0;  // roll amplitude (minor axis)
float B = 30.0;  // yaw amplitude (major axis)

float t = millis() / 1000.0; // time in seconds, adjust speed as needed

float roll_angle = A * sin(t);   // degrees
float yaw_angle  = B * cos(t);   // degrees

// Map from angle range (-max to +max) to servo 0–180
int roll_servo_cmd = map(roll_angle, -66.712, 72.05, 0, 180);
int yaw_servo_cmd  = map(yaw_angle, -39.65, 34.6, 0, 180);

// Write to servos
roll_servo.write(roll_servo_cmd);
yaw_servo.write(yaw_servo_cmd);


  // put your main code here, to run repeatedly:
  // roll to do top to bottom: 0, 45, 90, 135, 180
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

  // // yaw do left to right: 0, 45, 90, 135, 180
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
}
