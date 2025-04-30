#include <ESP32Servo.h>
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

const int steps = 200;
const float pi = 3.14159;

float pitchAngles[steps];
float yawAngles[steps];

// extra button stuff for deliverable
volatile bool buttonPressed = false;

void IRAM_ATTR isr() {  // the function to be called when interrupt is triggered
  if (buttonPressed) {
    buttonPressed = false;
  } else {
    buttonPressed = true;
  }

  Serial.print("Button Pressed:");
  Serial.println(buttonPressed);
}

// Ellipse dimensions (in virtual mm space)
float a = 80.0;   // major axis (pitch direction)
float b = 25.0;   // minor axis (yaw direction)
float c = 20.0;   // vertical depth arc to create the "C" shape

// void generatePath() { // works well but sputters at the ends
//   for (int i = 0; i < steps; i++) {
//     float t = map(i, 0, steps - 1, 0.0, 2 * pi * 1000) / 1000.0;

//     // Desired point in space (ellipse with forward curve)
//     float z = a * cos(t);        // forward/backward (pitch)
//     float x = b * sin(t);        // side to side (yaw)
//     float y = c * sin(t / 2.0);  // height variation ("C" shape)

//     // Compute pitch angle (tilting toward point in Y-Z plane)
//     float pitchRad = atan2(y, z);  // can now go negative or beyond vertical
//     float pitchDeg = degrees(pitchRad);

//     // Shift from [-90, +90] to [0, 180]
//     pitchAngles[i] = constrain(pitchDeg, 0.0, 180.0);

//     // Compute yaw based on X displacement relative to forward direction
//     float r_pitch = sqrt(y * y + z * z);  // length in Y-Z plane
//     float yawRad = atan2(x, r_pitch);
//     float yawDeg = degrees(yawRad);

//     yawAngles[i] = constrain(yawDeg + 90.0, 0.0, 180.0);
//   }
// }

// void generatePath() {
//   for (int i = 0; i < steps; i++) {
//     float linearT = (float)i / (steps - 1);           // Goes from 0 to 1
//     float t = (1 - cos(linearT * PI)) / 2.0 * 2 * PI; // Smooth cosine ramp [0, 2π]

//     float z = a * cos(t);        // forward/backward (pitch)
//     float x = b * sin(t);        // side to side (yaw)
//     float y = c * sin(t / 2.0);  // height variation

//     // Pitch computation
//     float pitchRad = atan2(y, z);
//     float pitchDeg = degrees(pitchRad);
//     pitchAngles[i] = constrain(pitchDeg, 0.0, 180.0);

//     // Yaw computation
//     float r_pitch = sqrt(y * y + z * z);
//     float yawRad = atan2(x, r_pitch);
//     float yawDeg = degrees(yawRad);
//     yawAngles[i] = constrain(yawDeg + 90.0, 0.0, 180.0);
//   }
// }

// VERSION 2, starts and ends at streamlined position
void generatePath() {
  for (int i = 0; i < steps; i++) {
    float linearT = (float)i / (steps - 1);              // Goes from 0 to 1
    float t = (1 - cos(linearT * PI)) / 2.0 * 2 * PI;    // Smooth cosine ramp [0, 2π]

    float z = a * cos(t);        // forward/backward (pitch)
    float x = b * sin(t);        // side to side (yaw)
    float y = c * sin(t / 2.0);  // height variation

    // Compute pitch and yaw
    float pitchRad = atan2(y, z);
    float pitchDeg = degrees(pitchRad);
    pitchDeg = constrain(pitchDeg, 0.0, 180.0);

    float r_pitch = sqrt(y * y + z * z);
    float yawRad = atan2(x, r_pitch);
    float yawDeg = degrees(yawRad);
    yawDeg = constrain(yawDeg + 90.0, 0.0, 180.0);

    // Smooth blend to fixed start and end values (ease-in-out curve)
    float blendT = (1 - cos(linearT * PI)) / 2.0;  // 0 at start, 1 at end, smooth
    float inverseBlend = 1.0 - blendT;

    // Fixed start/end constraints
    float startPitch = 45.0;
    float startYaw = 180.0;

    // Blend pitch and yaw toward start/end values at the endpoints
    pitchDeg = inverseBlend * startPitch + blendT * pitchDeg;
    yawDeg = inverseBlend * startYaw + blendT * yawDeg;

    pitchAngles[i] = pitchDeg;
    yawAngles[i] = yawDeg;

    // Compute delay for smooth movement
    float easingFactor = sin(i * PI / steps); // 0 at start/end, 1 at middle
    int delayTime = 10 + (int)(10 * (1 - easingFactor)); // 10–20 ms delay

    // total_time += delayTime;
  }
}

// Version 3 add limit so flipper is never pointing straight down
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

//     // --- Force pitch > 90° when yaw is near 90° ---
//     float yawCenter = 90.0;
//     float yawRange = 15.0;  // defines "near 90"
//     float yawDelta = abs(yawDeg - yawCenter);

//     if (yawDelta < yawRange) {
//       // Compute how strongly we should push pitch toward >90°
//       float strength = 1.0 - (yawDelta / yawRange);  // 1.0 when exactly 90°
//       float targetPitch = max(pitchDeg, float(90.0 + 30.0 * strength));  // raise pitch
//       pitchDeg = targetPitch;
//     }

//     // --- Apply start/end constraints as before ---
//     float blendT = (1 - cos(linearT * PI)) / 2.0;
//     float inverseBlend = 1.0 - blendT;
//     float startPitch = 45.0;
//     float startYaw = 180.0;
//     pitchDeg = inverseBlend * startPitch + blendT * pitchDeg;
//     yawDeg = inverseBlend * startYaw + blendT * yawDeg;

//     pitchAngles[i] = pitchDeg;
//     yawAngles[i] = yawDeg;

//     // Delay time calculation
//     float easingFactor = sin(i * PI / steps);
//     int delayTime = 10 + (int)(10 * (1 - easingFactor));
//     // total_time += delayTime;
//   }
// }

void setup() {
  Serial.begin(115200);

  R_roll_servo.attach(R_ROLL_PIN);
  R_yaw_servo.attach(R_YAW_PIN);
  R_pitch_servo.attach(R_PITCH_PIN);

  L_roll_servo.attach(L_ROLL_PIN);
  L_yaw_servo.attach(L_YAW_PIN);
  L_pitch_servo.attach(L_PITCH_PIN);

  // int BTN = 27;
  // pinMode(BTN, INPUT); // button attached to GPIO pin 27
  // digitalWrite(BTN, HIGH);
  // attachInterrupt(BTN, isr, FALLING);

  generatePath(); // Generate the path for the servos
}

void loop() {
    // wait for button press to start
    // if (buttonPressed) {

      // LEFT AND RIGHT at the same time
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

      // // JUST RIGHT FLIPPER
      // for (int i = steps - 1; i >= 0; i--) {
      // // clockwise looking from the right side
      //     L_roll_servo.write(pitchAngles[i]); // physically is pitch but on mine is roll, BASE SERVO
      //     L_yaw_servo.write(yawAngles[i]); // MIDDLE SERVO

      //     float easingFactor = sin(i * PI / steps); // 0 at start/end, 1 at middle
      //     int delayTime = 10 + (int)(10 * (1 - easingFactor)); // 10–20 ms delay

      //     delay(delayTime);
      // }

      // // JUST LEFT FLIPPER
      // for (int i = 0; i < steps; i++) {
      // // counterclockwise looking from the left side
      //     L_roll_servo.write(pitchAngles[i]); // physically is pitch but on mine is roll, BASE SERVO
      //     L_yaw_servo.write(yawAngles[i]); // MIDDLE SERVO

      //     float easingFactor = sin(i * PI / steps); // 0 at start/end, 1 at middle
      //     int delayTime = 10 + (int)(10 * (1 - easingFactor)); // 10–20 ms delay

      //     delay(delayTime);
      // }

    // }
}
