//// with rod end attached to end of yaw servo but should be roll servo
#include <ESP32Servo.h>

Servo roll_servo; // object representing actual servo
Servo yaw_servo;
Servo pitch_servo;

const int steps = 200;
float t_vals[steps];
float pitchAngles[steps];
float yawAngles[steps];

// Arm lengths and offsets (in mm)
const float L1 = 57.0;    // Base to pitch servo
const float L2 = 115.67;    // Pitch to yaw
const float L3 = 310.0;    // Yaw to rod tip

/////////////////////ANOTHER VERSION///////////////////////
// Timing
const float interval = 20; // ms between steps
float t = 0.0;

void setup() {
  Serial.begin(115200);
  pitch_servo.attach(4); // connected to GPIO pin 4
  yaw_servo.attach(18);
  roll_servo.attach(5);
  computeAngles();
}

void loop() {
  // for (int i = 0; i < steps; i++) {
  //   pitch_servo.write(pitchAngles[i]); // physically is pitch but on mine is roll
  //   yaw_servo.write(yawAngles[i]);

  //   if (i == 0) {
  //     Serial.println("_____START______");
  //   }
  //   Serial.print("Pitch: ");
  //   Serial.print(pitchAngles[i]);
  //   Serial.print(", Yaw: ");
  //   Serial.println(yawAngles[i]);
  //   delay(20); // smooth motion
  // }

  for (int i = 0; i < steps; i++) {
    // change direction
    float ti = t + (steps - i) * (2 * PI / steps);
    // float ti = t + i * (2 * PI / steps);

    // Path equations
    float xt = 117.1 - 182.9*cos(1.496*ti) + 42.63*sin(1.496*ti)
      - 13.3*cos(2*1.496*ti) - 54.58*sin(2*1.496*ti)
      + 17.38*cos(3*1.496*ti) - 8.86*sin(3*1.496*ti)
      - 5.189*cos(4*1.496*ti) + 1.512*sin(4*1.496*ti)
      - 2.687*cos(5*1.496*ti) - 2.955*sin(5*1.496*ti)
      + 2.653*cos(6*1.496*ti) - 2.801*sin(6*1.496*ti)
      - 0.3433*cos(7*1.496*ti) - 1.62*sin(7*1.496*ti)
      - 0.08304*cos(8*1.496*ti) - 1.225*sin(8*1.496*ti);

    float yt = 303 + 219.6*cos(1.496*ti) - 164.3*sin(1.496*ti)
      + 84.45*cos(2*1.496*ti) - 65.54*sin(2*1.496*ti)
      + 26.33*cos(3*1.496*ti) + 45.71*sin(3*1.496*ti)
      - 11.23*cos(4*1.496*ti) + 20.05*sin(4*1.496*ti)
      - 0.5956*cos(5*1.496*ti) - 2.631*sin(5*1.496*ti)
      - 2.667*cos(6*1.496*ti) + 0.6726*sin(6*1.496*ti)
      + 0.678*cos(7*1.496*ti) - 0.7038*sin(7*1.496*ti)
      + 2.542*cos(8*1.496*ti) - 0.9941*sin(8*1.496*ti);

    float zt = 262.7 + 11.65*cos(1.496*ti) - 63.16*sin(1.496*ti)
      - 33.56*cos(2*1.496*ti) + 82.36*sin(2*1.496*ti)
      - 38.04*cos(3*1.496*ti) + 23.7*sin(3*1.496*ti)
      - 14.84*cos(4*1.496*ti) - 8.802*sin(4*1.496*ti)
      - 7.894*cos(5*1.496*ti) - 4.65*sin(5*1.496*ti)
      - 6.884*cos(6*1.496*ti) - 6.088*sin(6*1.496*ti)
      + 2.615*cos(7*1.496*ti) - 3.547*sin(7*1.496*ti)
      - 1.716*cos(8*1.496*ti) + 1.293*sin(8*1.496*ti);

    // Inverse kinematics:
    float dx = xt;
    float dy = yt;
    float dz = zt - L1;

    // Compute pitch angle (elevation in Y-Z plane)
    float pitchRad = atan2(dy, sqrt(dx*dx + dz*dz));
    float pitchDeg = constrain(pitchRad * 180.0 / PI, 0, 180);

    // Compute yaw angle (in X-Z plane)
    float yawRad = atan2(dx, dz);
    float yawDeg = constrain(yawRad * 180.0 / PI, 0, 180);

    // Roll angle: optional rotation around rod axis — smooth cycle
    float rollDeg = 90 + 30 * sin(ti);  // Just some pleasant motion

    if (i == 0) {
      Serial.println("_____START______");
    }
    Serial.print("Pitch: ");
    Serial.print(pitchDeg);
    Serial.print(", Yaw: ");
    Serial.print(yawDeg);
    Serial.print(", Roll: ");
    Serial.println(rollDeg);

    // Write to servos
    pitch_servo.write(pitchDeg);
    yaw_servo.write(yawDeg);
    roll_servo.write(rollDeg);

    delay(interval);
  }

  // Loop again
  t += 2 * PI;
}

// ----------------------- Core Calculation --------------------------

void computeAngles() {
  for (int i = 0; i < steps; i++) {
    float t = map(i, 0, steps - 1, 0, 1000) / 1000.0 * TWO_PI;
    float wt = t * 1.496;

    float xt = 117.1 - 182.9 * cos(wt) + 42.63 * sin(wt)
             - 13.3 * cos(2 * wt) - 54.58 * sin(2 * wt)
             + 17.38 * cos(3 * wt) - 8.86 * sin(3 * wt)
             - 5.189 * cos(4 * wt) + 1.512 * sin(4 * wt)
             - 2.687 * cos(5 * wt) - 2.955 * sin(5 * wt)
             + 2.653 * cos(6 * wt) - 2.801 * sin(6 * wt)
             - 0.3433 * cos(7 * wt) - 1.62 * sin(7 * wt)
             - 0.08304 * cos(8 * wt) - 1.225 * sin(8 * wt);

    float yt = 303 + 219.6 * cos(wt) - 164.3 * sin(wt)
             + 84.45 * cos(2 * wt) - 65.54 * sin(2 * wt)
             + 26.33 * cos(3 * wt) + 45.71 * sin(3 * wt)
             - 11.23 * cos(4 * wt) + 20.05 * sin(4 * wt)
             - 0.5956 * cos(5 * wt) - 2.631 * sin(5 * wt)
             - 2.667 * cos(6 * wt) + 0.6726 * sin(6 * wt)
             + 0.678 * cos(7 * wt) - 0.7038 * sin(7 * wt)
             + 2.542 * cos(8 * wt) - 0.9941 * sin(8 * wt);

    float zt = 262.7 + 11.65 * cos(wt) - 63.16 * sin(wt)
             - 33.56 * cos(2 * wt) + 82.36 * sin(2 * wt)
             - 38.04 * cos(3 * wt) + 23.7 * sin(3 * wt)
             - 14.84 * cos(4 * wt) - 8.802 * sin(4 * wt)
             - 7.894 * cos(5 * wt) - 4.65 * sin(5 * wt)
             - 6.884 * cos(6 * wt) - 6.088 * sin(6 * wt)
             + 2.615 * cos(7 * wt) - 3.547 * sin(7 * wt)
             - 1.716 * cos(8 * wt) + 1.293 * sin(8 * wt);

    // Convert coordinates to local relative frame
    float x = xt;
    float y = yt - L1; // Offset from base to pitch servo
    float z = zt;

    // Inverse Kinematics for pitch and yaw
    float pitchRad = atan2(sqrt(x * x + z * z), y);
    float yawRad = atan2(x, z);

    // Convert to degrees and offset
    float pitchDeg = constrain(degrees(pitchRad), 0.0, 180.0);
    float yawDeg = constrain(degrees(yawRad) + 90.0, 0.0, 180.0);

    pitchAngles[i] = pitchDeg;
    yawAngles[i] = yawDeg;
  }
}
