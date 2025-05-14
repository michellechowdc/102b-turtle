#include <ESP32Servo.h>

Servo servo1, servo2, servo3;
const int pin1 = 25;
const int pin2 = 26;
// const int pin3 = 11;

String inputString = "";
bool inputComplete = false;

void setup() {
  Serial.begin(115200);
  servo1.attach(pin1);
  servo2.attach(pin2);
  // servo3.attach(pin3);

  Serial.println("Enter angles like: S1:90 S2:45 S3:120");
}

void loop() {
  if (inputComplete) {
    parseAndMoveServos(inputString);
    inputString = "";
    inputComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      inputComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void parseAndMoveServos(String input) {
  input.trim();  // remove any leading/trailing whitespace

  int s1 = parseServoAngle(input, "S1:");
  int s2 = parseServoAngle(input, "S2:");
  // int s3 = parseServoAngle(input, "S3:");

  if (s1 >= 0) {
    servo1.write(s1);
    Serial.print("S1 set to "); Serial.println(s1);
  }
  if (s2 >= 0) {
    servo2.write(s2);
    Serial.print("S2 set to "); Serial.println(s2);
  }
  // if (s3 >= 0) {
  //   servo3.write(s3);
  //   Serial.print("S3 set to "); Serial.println(s3);
  // }
}

int parseServoAngle(String input, String tag) {
  int idx = input.indexOf(tag);
  if (idx == -1) return -1;

  int start = idx + tag.length();
  int end = input.indexOf(' ', start);
  if (end == -1) end = input.length();

  String valueStr = input.substring(start, end);
  int angle = valueStr.toInt();

  if (angle >= 0 && angle <= 180) {
    return angle;
  } else {
    Serial.print("Invalid angle for "); Serial.print(tag); Serial.println(" Skipping.");
    return -1;
  }
}
