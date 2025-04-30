#include <ESP32Servo.h>

// replace end servos for joystick pins for now

////////////////////////////////////
////// SETUP FRONT FLIPPERS ////////
Servo R_roll_servo; // object representing actual servo
Servo R_yaw_servo;
// Servo R_pitch_servo;
const int R_ROLL_PIN = 33; // base servo, connected to body
const int R_YAW_PIN = 27; // middle servo
// const int R_PITCH_PIN = 12; // end servo, connected to flipper

Servo L_roll_servo;
Servo L_yaw_servo;
// Servo L_pitch_servo;
const int L_ROLL_PIN = 14; // base servo, connected to body
const int L_YAW_PIN = 32; // middle servo
// const int L_PITCH_PIN = 15; // end servo, connected to flipper

const int steps = 200;
const float pi = 3.14159;

float pitchAngles[steps];
float yawAngles[steps];

// Ellipse dimensions (in virtual mm space)
float a = 80.0;   // major axis (pitch direction)
float b = 25.0;   // minor axis (yaw direction)
float c = 20.0;   // vertical depth arc to create the "C" shape

int total_time = 0; // total time needed for the path

///////////////// program state
int state = 2;

///////////////// joystick code
const int xPin = 12;
const int yPin = 15;

///////////////// button code
const int buttonPin = 13; 
volatile bool buttonIsPressed = false;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // 50 milliseconds

void IRAM_ATTR btn_isr() {  // the function to be called when interrupt is triggered
  unsigned long currentTime = millis();
  // Check if enough time has passed since the last press
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    buttonIsPressed = true;
    lastDebounceTime = currentTime;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(buttonPin, INPUT);  // configures the specified pin to behave either as an input or an output
  attachInterrupt(buttonPin, btn_isr, FALLING);

  // front flippers
  R_roll_servo.attach(R_ROLL_PIN);
  R_yaw_servo.attach(R_YAW_PIN);
  // R_pitch_servo.attach(R_PITCH_PIN);

  L_roll_servo.attach(L_ROLL_PIN);
  L_yaw_servo.attach(L_YAW_PIN);
  // L_pitch_servo.attach(L_PITCH_PIN);

  generatePath(); // Generate the path for the servos
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);

  // read joystick values
  int xVal = analogRead(xPin);  // Read the X-axis
  int yVal = analogRead(yPin);  // Read the Y-axis

  switch(state) {
    case 1: // Turtle Off
      if (event_ButtonPressed()) {
        state = 2;
      } else {
        Serial.println("Turtle Off (Button not pressed)");
      }

      break;
    case 2: // Turtle On
      // if (event_ButtonPressed()) {
      //   state = 1;
      // } else 
      if (yVal < 3110) {
        state = 3;
      } else {
        Serial.println("Turtle On (Button Pressed)");
      }

      break;
    case 3: // Move Forward
      if (event_ButtonPressed()) {
        state = 1;
      } else if (yVal >= 3110) {
        state = 2;
      } else if (xVal > 3250) {
        state = 4;
      } else if (xVal < 3230) {
        state = 5;
      } else {
        Serial.println("Move Forward");
        service_frontForward();
        service_backNeutral();
      }

      break;
    case 4: // Move Right
      if (event_ButtonPressed()) {
        state = 1;
      } else if (xVal <= 3250) {
        state = 3;
      } else {
        Serial.println("Move Right");
        service_frontForward();
        service_backRight();
      }

      break;
    case 5: // Move Left
      if (event_ButtonPressed()) {
        state = 1;
      } else if (xVal >= 3230) {
        state = 3;
      } else {
        Serial.println("Move Left");
        service_frontForward();
        service_backLeft();
      }

      break;
  }
}

bool event_ButtonPressed() {
  if (buttonIsPressed == true) {
    buttonIsPressed = false;
    return true;
  } else {
    return false;
  }
}

void service_frontForward() {
  Serial.println("Moving Forward");

  int right_i = steps;
  for (int left_i = 0; left_i < steps; left_i++) {
    // counterclockwise looking from the left side
    L_roll_servo.write(pitchAngles[left_i]); // physically is pitch but on mine is roll, BASE SERVO
    L_yaw_servo.write(yawAngles[left_i]); // MIDDLE SERVO

    // clockwise looking from the right side
    right_i--;
    // R_roll_servo.write(pitchAngles[right_i]); // physically is pitch but on mine is roll, BASE SERVO
    // R_yaw_servo.write(yawAngles[right_i]); // MIDDLE SERVO

    float easingFactor = sin(left_i * PI / steps); // 0 at start/end, 1 at middle
    int delayTime = 10 + (int)(10 * (1 - easingFactor)); // 10–20 ms delay

    delay(delayTime);
  }

}

void service_backNeutral() {
  Serial.println("Moving Neutral");
}

void service_backRight() {
  Serial.println("Moving Right");
}

void service_backLeft() {
  Serial.println("Moving Left");
}

void generatePath() {
  for (int i = 0; i < steps; i++) {
    float linearT = (float)i / (steps - 1);           // Goes from 0 to 1
    float t = (1 - cos(linearT * PI)) / 2.0 * 2 * PI; // Smooth cosine ramp [0, 2π]

    float z = a * cos(t);        // forward/backward (pitch)
    float x = b * sin(t);        // side to side (yaw)
    float y = c * sin(t / 2.0);  // height variation

    // Pitch computation
    float pitchRad = atan2(y, z);
    float pitchDeg = degrees(pitchRad);
    pitchAngles[i] = constrain(pitchDeg, 0.0, 180.0);

    // Yaw computation
    float r_pitch = sqrt(y * y + z * z);
    float yawRad = atan2(x, r_pitch);
    float yawDeg = degrees(yawRad);
    yawAngles[i] = constrain(yawDeg + 90.0, 0.0, 180.0);

    // Compute total time for the path
    float easingFactor = sin(i * PI / steps); // 0 at start/end, 1 at middle
    int delayTime = 10 + (int)(10 * (1 - easingFactor)); // 10–20 ms delay

    total_time += delayTime; // accumulate the total time
  }
}
