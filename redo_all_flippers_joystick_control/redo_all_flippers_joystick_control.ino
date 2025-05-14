#include <ESP32Servo.h>

// replace end servos for joystick pins for now

////////////////////////////////////
////// SETUP FRONT FLIPPERS ////////
Servo R_roll_servo; // object representing actual servo
Servo R_yaw_servo;
const int R_ROLL_PIN = 14; // base servo, connected to body
const int R_YAW_PIN = 32; // middle servo

Servo L_roll_servo;
Servo L_yaw_servo;
const int L_ROLL_PIN = 25; // base servo, connected to body
const int L_YAW_PIN = 26; // middle servo

const int steps = 200;
const float pi = 3.14159;

float pitchAngles[steps];
float yawAngles[steps];

// Ellipse dimensions (in virtual mm space)
float a = 80.0;   // major axis (pitch direction)
float b = 25.0;   // minor axis (yaw direction)
float c = 20.0;   // vertical depth arc to create the "C" shape

int total_time = 0; // total time needed for the path

////////////////////////////////////
////// SETUP BACK FLIPPERS /////////
const int bR_rollPin = 4;
Servo bR_rollServo;

const int bL_rollPin = 5;
Servo bL_rollServo;

///////////////// program state
int state = 2;

///////////////// joystick code
const int xPin = 13;
const int yPin = 12;

///////////////// button code
const int buttonPin = 27; 
volatile bool buttonIsPressed = false;

///////////////// LED code
const int redLEDPin = 33;
const int yelLEDPin = 15;
volatile bool LEDon = false;
// volatile bool i = false;

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

  //// button setup
  pinMode(buttonPin, INPUT);  // configures the specified pin to behave either as an input or an output
  attachInterrupt(buttonPin, btn_isr, FALLING);

  ///// front flippers
  R_roll_servo.attach(R_ROLL_PIN);
  R_yaw_servo.attach(R_YAW_PIN);

  L_roll_servo.attach(L_ROLL_PIN);
  L_yaw_servo.attach(L_YAW_PIN);

  ///// back flippers
  bR_rollServo.attach(bR_rollPin);
  bL_rollServo.attach(bL_rollPin);

  ///// LEDs
  pinMode(redLEDPin, OUTPUT);
  pinMode(yelLEDPin, OUTPUT);

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
      // service_blink();
      // LEDon = blink_check();
      
      if (event_ButtonPressed()) {
        digitalWrite(redLEDPin, HIGH);
        digitalWrite(yelLEDPin, HIGH);
        state = 2;
      } else {
        Serial.println("Turtle Off (Button not pressed)");
      }

      break;
    case 2: // Turtle On

      // if (event_ButtonPressed()) {
      //   service_LEDoff();
      //   state = 1;
      // } else 
      if (event_moveForward(yVal)) {
        service_LEDoff();
        state = 3;
      } else {
        Serial.println("Turtle On (Button Pressed)");
      }

      break;
    case 3: // Move Forward
      // if (event_ButtonPressed()) {
      //   service_LEDoff();
      //   state = 1;
      // } else 
      if (!event_moveForward(yVal)) {
        service_LEDon();
        state = 2;
      } else if (event_moveRight(xVal)) {
        state = 4;
      } else if (event_moveLeft(xVal)) {
        state = 5;
      } else {
        Serial.println("Move Forward");
        service_LEDon();
        service_backNeutral();
        service_frontForward();
      }

      break;
    case 4: // Move Right
      // if (event_ButtonPressed()) {
      //   service_LEDoff();
      //   state = 1;
      // } else 
      if (!event_moveRight(xVal)) {
        service_LEDon();
        state = 3;
      } else {
        Serial.println("Move Right");
        service_backRight();
        service_frontForward();
      }

      break;
    case 5: // Move Left
      // if (event_ButtonPressed()) {
      //   service_LEDoff();
      //   state = 1;
      // } else 
      if (!event_moveLeft(xVal)) {
        service_LEDon();
        state = 3;
      } else {
        Serial.println("Move Left");
        service_backLeft();
        service_frontForward();
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

bool event_moveForward(int yVal) {
  // y threshold is 3110
  // y ranges from 0 to 4095, center is 3115
  return (yVal < 3010);
}

bool event_moveRight(int xVal) {
  // x threshold is 3250
  // x ranges from 0 to 4095, center is 3230
  digitalWrite(yelLEDPin, HIGH);
  digitalWrite(redLEDPin, LOW);
  return (xVal == 4095);
}

bool event_moveLeft(int xVal) {
  // x threshold is 3230
  // x ranges from 0 to 4095, center is 3230
  digitalWrite(redLEDPin, HIGH);
  digitalWrite(yelLEDPin, LOW);
  return (xVal == 0);
}

void service_frontForward() {
  Serial.println("Moving Forward");

  int right_i = steps;
  for (int left_i = 0; left_i < steps; left_i++) {
    // counterclockwise looking from the left side
    L_roll_servo.write(pitchAngles[left_i]); // physically is pitch but on mine is roll, BASE SERVO
    L_yaw_servo.write(yawAngles[right_i]); // MIDDLE SERVO

    // clockwise looking from the right side
    right_i--;
    R_roll_servo.write(pitchAngles[right_i]); // physically is pitch but on mine is roll, BASE SERVO
    R_yaw_servo.write(yawAngles[left_i]); // MIDDLE SERVO

    // R_roll_servo.write(pitchAngles[left_i]); // physically is pitch but on mine is roll, BASE SERVO
    // R_yaw_servo.write(yawAngles[left_i]); // MIDDLE SERVO

    // float easingFactor = sin(left_i * PI / steps); // 0 at start/end, 1 at middle
    // int delayTime = 10 + (int)(10 * (1 - easingFactor)); // 10–20 ms delay

    // delay(delayTime);
    delay(10);
  }

}

void service_backNeutral() {
  Serial.println("Moving Neutral");
  bL_rollServo.write(90);
  bR_rollServo.write(90);
}

void service_backRight() {
  Serial.println("Moving Right");
  bL_rollServo.write(90);
  bR_rollServo.write(130);
}

void service_backLeft() {
  Serial.println("Moving Left");
  bL_rollServo.write(60);
  bR_rollServo.write(90);
}

void service_blink() {
  digitalWrite(redLEDPin, HIGH);  // turn the LED on (HIGH is the voltage level)
  digitalWrite(yelLEDPin, HIGH);   // turn the LED off by making the voltage LOW
  delay(50);                      // wait for a second
  digitalWrite(redLEDPin, LOW);   // turn the LED off by making the voltage LOW
  digitalWrite(yelLEDPin, LOW);  // turn the LED on (HIGH is the voltage level)
  delay(50);                      // wait for a second
}

bool blink_check() {
  if (LEDon == true) {
    LEDon = false;
    return true;
  } else {
    return false;
  }
}

void service_LEDoff() {
  digitalWrite(redLEDPin, LOW);   // turn the LED off by making the voltage LOW
  digitalWrite(yelLEDPin, LOW);  // turn the LED on (HIGH is the voltage level)
}

void service_LEDon() {
  digitalWrite(redLEDPin, HIGH);   // turn the LED off by making the voltage LOW
  digitalWrite(yelLEDPin, HIGH);  // turn the LED on (HIGH is the voltage level)
}

// move to Ashley's Ballast code
// void service_backUp() {
//   Serial.println("Moving Up");
//   yawServo.write(0);
//   pitchServo.write(45);
//   rollServo.write(90);
// }

// void service_backDown() {
//   Serial.println("Moving Down");
//   neutralBackFlipper();
// }
/////////

///////////// swim stroke
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

///////////// walking path
// manually program a path
// void generatePath() {
//   for (int i = 0; i < steps; i++) {
//     // for pitch angles, first 100 at 130deg then 150deg
//     if (i < 100) {
//       pitchAngles[i] = 130.0;
//     } else {
//       pitchAngles[i] = 150.0;
//     }

//     // for yaw angles, evenly increment from 150 to 0deg in first 100 steps
//     // then decrement from 0 to 150deg in next 100
//     float increment = 150 / 100;
//     if (i == 0) {
//       yawAngles[0] = 150;
//     } else if (i < 100) {
//       yawAngles[i] = yawAngles[i - 1] - increment;
//     } else {
//       yawAngles[i] = yawAngles[i - 1] + increment;
//     }
//   }
// }

///////////// walking path
// void generatePath() {
//   for (int i = 0; i < steps; i++) {
//     float linearT = (float)i / (steps - 1);           // Goes from 0 to 1
//     float t = (1 - cos(linearT * PI)) / 2.0;          // Smooth cosine ramp [0,1]

//     // CONTROLLED "C" PATH IN SIDE VIEW:
//     float z = -a * cos(t * PI);       // forward-backward (pitch)
//     float y = c * sin(t * PI);        // up-down (lift)

//     // TEARDROP-SHAPED PATH IN TOP VIEW:
//     // outward during lift, back along side during power
//     float x;
//     if (t < 0.5) {
//       x = b * sin(t * 2 * PI);        // lift phase: go outward
//     } else {
//       x = b * (1 - 2 * (t - 0.5));    // return phase: gradually come inward
//     }

//     // Compute pitch angle
//     float pitchRad = atan2(y, z);
//     float pitchDeg = degrees(pitchRad);
//     pitchAngles[i] = constrain(pitchDeg, 0.0, 180.0);

//     // Compute yaw angle
//     float r_pitch = sqrt(y * y + z * z);
//     float yawRad = atan2(x, r_pitch);
//     float yawDeg = degrees(yawRad);
//     yawAngles[i] = constrain(yawDeg + 90.0, 0.0, 180.0);  // 90° is forward

//     // Compute timing
//     float easingFactor = sin(i * PI / steps); // 0 at ends, 1 at middle
//     int delayTime = 10 + (int)(10 * (1 - easingFactor)); // 10–20 ms
//     total_time += delayTime;
//   }
// }

// adjusted
// void generatePath() {
//   for (int i = 0; i < steps; i++) {
//     float linearT = (float)i / (steps - 1);              // Goes from 0 to 1
//     float t = (1 - cos(linearT * PI)) / 2.0 * 2 * PI;    // Smooth cosine ramp [0, 2π]

//     float z = a * cos(t);        // forward/backward (pitch)
//     float x = b * sin(t);        // side to side (yaw)
//     float y = c * sin(t / 2.0);  // height variation

//     // Compute pitch and yaw
//     float pitchRad = atan2(y, z);
//     float pitchDeg = degrees(pitchRad);
//     pitchDeg = constrain(pitchDeg, 0.0, 180.0);

//     float r_pitch = sqrt(y * y + z * z);
//     float yawRad = atan2(x, r_pitch);
//     float yawDeg = degrees(yawRad);
//     yawDeg = constrain(yawDeg + 90.0, 0.0, 180.0);

//     // Smooth blend to fixed start and end values (ease-in-out curve)
//     float blendT = (1 - cos(linearT * PI)) / 2.0;  // 0 at start, 1 at end, smooth
//     float inverseBlend = 1.0 - blendT;

//     // Fixed start/end constraints
//     float startPitch = 45.0;
//     float startYaw = 180.0;

//     // Blend pitch and yaw toward start/end values at the endpoints
//     pitchDeg = inverseBlend * startPitch + blendT * pitchDeg;
//     yawDeg = inverseBlend * startYaw + blendT * yawDeg;

//     pitchAngles[i] = pitchDeg;
//     yawAngles[i] = yawDeg;

//     // Compute delay for smooth movement
//     float easingFactor = sin(i * PI / steps); // 0 at start/end, 1 at middle
//     int delayTime = 10 + (int)(10 * (1 - easingFactor)); // 10–20 ms delay

//     total_time += delayTime;
//   }
// }
