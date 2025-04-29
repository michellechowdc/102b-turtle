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
