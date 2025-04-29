#include <Servo.h>

Servo roll_servo; // object representing actual servo
Servo yaw_servo;
Servo pitch_servo;

const float L1 = 57.0;
const float L2 = 115.67;
const float L3 = 310.0;

const int numPoints = 100;
float tVals[numPoints];

void setup() {
  pitch_servo.attach(4);
  yaw_servo.attach(18);
  roll_servo.attach(5);

  for (int i = 0; i < numPoints; i++) {
    tVals[i] = TWO_PI - i * TWO_PI / (numPoints - 1);  // Reverse direction
  }
}

void loop() {
  for (int i = 0; i < numPoints; i++) {
    float t = tVals[i];
    float w = 1.496;

    float xt =  117.1 - 182.9*cos(w*t) + 42.63*sin(w*t)
                - 13.3*cos(2*w*t) - 54.58*sin(2*w*t)
                + 17.38*cos(3*w*t) - 8.86*sin(3*w*t)
                - 5.189*cos(4*w*t) + 1.512*sin(4*w*t)
                - 2.687*cos(5*w*t) - 2.955*sin(5*w*t)
                + 2.653*cos(6*w*t) - 2.801*sin(6*w*t)
                - 0.3433*cos(7*w*t) - 1.62*sin(7*w*t)
                - 0.08304*cos(8*w*t) - 1.225*sin(8*w*t);

    float yt =  303 + 219.6*cos(w*t) - 164.3*sin(w*t)
                + 84.45*cos(2*w*t) - 65.54*sin(2*w*t)
                + 26.33*cos(3*w*t) + 45.71*sin(3*w*t)
                - 11.23*cos(4*w*t) + 20.05*sin(4*w*t)
                - 0.5956*cos(5*w*t) - 2.631*sin(5*w*t)
                - 2.667*cos(6*w*t) + 0.6726*sin(6*w*t)
                + 0.678*cos(7*w*t) - 0.7038*sin(7*w*t)
                + 2.542*cos(8*w*t) - 0.9941*sin(8*w*t);

    float zt =  262.7 + 11.65*cos(w*t) - 63.16*sin(w*t)
                - 33.56*cos(2*w*t) + 82.36*sin(2*w*t)
                - 38.04*cos(3*w*t) + 23.7*sin(3*w*t)
                - 14.84*cos(4*w*t) - 8.802*sin(4*w*t)
                - 7.894*cos(5*w*t) - 4.65*sin(5*w*t)
                - 6.884*cos(6*w*t) - 6.088*sin(6*w*t)
                + 2.615*cos(7*w*t) - 3.547*sin(7*w*t)
                - 1.716*cos(8*w*t) + 1.293*sin(8*w*t);

    // Offset: transform global tip position into inverse kinematics space
    float Px = xt;
    float Py = yt - L1;  // Y minus base height
    float Pz = zt;

    float D = sqrt(Px*Px + Py*Py + Pz*Pz);

    float pitchAngle = atan2(Py, sqrt(Px*Px + Pz*Pz));
    float yawAngle   = atan2(Px, Pz);
    float rollAngle  = atan2(Py, Px);  // simplified placeholder

    // Map radians to degrees and servo range (0-180)
    int pitchDeg = constrain(map(pitchAngle * 180.0 / PI, 0, 180, 0, 180), 0, 180);
    int yawDeg   = constrain(map(yawAngle * 180.0 / PI + 90, 0, 180, 0, 180), 0, 180);
    int rollDeg  = constrain(map(rollAngle * 180.0 / PI + 90, 0, 180, 0, 180), 0, 180);

    pitch_servo.write(pitchDeg);
    yaw_servo.write(yawDeg);
    roll_servo.write(rollDeg);

    delay(20);  // smooth movement
  }
}
