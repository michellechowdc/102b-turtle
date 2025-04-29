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

////////////////////////////////////
////// SETUP BACK FLIPPERS /////////
const int bR_yawPin = 26; // might not be the actual order
const int bR_pitchPin = 25;
const int bR_rollPin = 34;

Servo bR_yawServo;
Servo bR_pitchServo;
Servo bR_rollServo;

const int bL_yawPin = 39;
const int bL_pitchPin = 36;
const int bL_rollPin = 4;

Servo bL_yawServo;
Servo bL_pitchServo;
Servo bL_rollServo;

////////////////////////////////////
////// SETUP JOYSTICK //////////////
const int xPin = 12; 
const int yPin = 15; 

const int buttonPin = 37; 

const int deadZone = 100;  // joystick values within this range from middle will be ignored

// previous servo positions
int prevYaw = -1;
int prevPitch = -1;

// joystick reading
int xValue, yValue, xMovement, yMovement, yaw, pitch;

////////////////////////////////////
// timer interrupt (lab 2)
hw_timer_t *timer = NULL;
volatile bool interruptCounter = false; 
int totalInterrupts = 0; 
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR timerISR() { // event checker
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void TimerInterruptInit() {
  timer = timerBegin(1000000); // 1 MHz
  timerAttachInterrupt(timer, &timerISR); 
  timerAlarm(timer, 1000000, true, 0);
}

void setup() {
  Serial.begin(115200);

  // front flippers
  R_roll_servo.attach(R_ROLL_PIN);
  R_yaw_servo.attach(R_YAW_PIN);
  // R_pitch_servo.attach(R_PITCH_PIN);

  L_roll_servo.attach(L_ROLL_PIN);
  L_yaw_servo.attach(L_YAW_PIN);
  // L_pitch_servo.attach(L_PITCH_PIN);

  generatePath(); // Generate the path for the servos

  // back flippers
  bR_yawServo.setPeriodHertz(50);  
  bR_yawServo.attach(bR_yawPin);

  bR_pitchServo.setPeriodHertz(50);
  bR_pitchServo.attach(bR_pitchPin);

  bR_rollServo.setPeriodHertz(50);
  bR_rollServo.attach(bR_rollPin);

  bL_yawServo.setPeriodHertz(50);  
  bL_yawServo.attach(bL_yawPin);

  bL_pitchServo.setPeriodHertz(50);
  bL_pitchServo.attach(bL_pitchPin);

  bL_rollServo.setPeriodHertz(50);
  bL_rollServo.attach(bL_rollPin);

  pinMode(buttonPin, INPUT_PULLUP);

  Serial.println("Enter yaw (theta), pitch (phi), and roll degrees (0-180) in format: yaw pitch roll");

  TimerInterruptInit();
}

void loop() {
  if (interruptCounter) { // if timer is interrupted every 50ms
    portENTER_CRITICAL(&timerMux);
    interruptCounter = false;  // Reset the flag to false
    portEXIT_CRITICAL(&timerMux);

    // Read joystick values
    xValue = analogRead(xPin);
    yValue = analogRead(yPin);

    // Check if the joystick is in the dead zone or not for both X and Y
    xMovement = abs(xValue - 3000) > deadZone ? 1 : 0;
    yMovement = abs(yValue - 3100) > deadZone ? 1 : 0;

    // button state (0 pressed, 1 released)
    int buttonState = digitalRead(buttonPin); // event checker
    handleButtonState(buttonState); // service function
  }
}

// Function to handle button state and joystick input or serial input
void handleButtonState(int buttonState) {
  Serial.println("In handleButtonState event checker");
  switch (buttonState) { // case switch block
    case LOW:  // button pressed
      Serial.println("Button is pressed");
      // Handle joystick movement for yaw and pitch
      handleJoystickMovement(xMovement, yMovement, xValue, yValue); // service function
      break;

    case HIGH:  // button not pressed
      Serial.println("Button is not pressed");
      handleSerialInput(); //service function
      break;
  }
}

// int checkJoystickMovement(int joystickValue) { // event checker
//   Serial.println("In checkJoystickMovement");
//   return abs(joystickValue - 3000) > deadZone ? 1 : 0;
// }

void handleJoystickMovement(int xMovement, int yMovement, int xValue, int yValue) { // service function
  Serial.println("In handleJoystickMovement service function");

  if (xMovement == 1) {
    yaw = map(xValue, 0, 4095, 0, 180);
    
    if (abs(yaw - prevYaw) > 2) { // look for more than 2 deg movement
      bR_yawServo.write(yaw); 
      bL_yawServo.write(yaw); 
      prevYaw = yaw;  // set prevyaw to previous yaw so if there is no update use that instead of resetting to 0
      Serial.print("Yaw (theta): ");
      Serial.println(yaw);
    }
  }

  if (yMovement == 1) {
    pitch = map(yValue, 0, 4095, 45, 180); // physical boundary at less than 45 deg
    
    if (abs(pitch - prevPitch) > 2) { 
      bR_pitchServo.write(pitch); 
      bL_pitchServo.write(pitch); 
      prevPitch = pitch;     
      Serial.print("Pitch (phi): ");
      Serial.println(pitch);
    }

    // Run handleFrontFlipperMovement() when joystick is moved up
    Serial.println("Joystick moved up, running handleFrontFlipperMovement...");
    handleFrontFlipperMovement();
    
    // Add a delay to ensure the function completes before checking joystick again
    delay(total_time);
  }
}

void handleSerialInput() { // service function
  Serial.println("In handleSerialInput service function");
  if (Serial.available() > 0) {
    int yaw = Serial.parseInt();  
    int pitch = Serial.parseInt(); 
    int roll = Serial.parseInt();  

    if (yaw >= 0 && yaw <= 180 && roll >= 0 && roll <= 180) {
    
      if (pitch < 45) {
        pitch = 45;  
      }
      if (pitch > 180) {
        pitch = 180; 
      }

      bR_yawServo.write(yaw);
      bL_yawServo.write(yaw);
      bR_pitchServo.write(pitch);
      bL_pitchServo.write(pitch);
      bR_rollServo.write(roll);  
      bL_rollServo.write(roll);  

      Serial.print("Yaw (theta): ");
      Serial.print(yaw);
      Serial.print(" | Pitch (phi): ");
      Serial.print(pitch);
      Serial.print(" | Roll: ");
      Serial.println(roll);
    } else {
      Serial.println("Please enter valid angles (0-180) for yaw, pitch, and roll.");
    }
  }
}

void handleFrontFlipperMovement() {
  Serial.println("In handleFrontFlipperMovement service function");

  // LEFT AND RIGHT at the same time
  int right_i = steps;
  for (int left_i = 0; left_i < steps; left_i++) {
    // counterclockwise looking from the left side
    L_roll_servo.write(pitchAngles[left_i]); // physically is pitch but on mine is roll, BASE SERVO
    L_yaw_servo.write(yawAngles[left_i]); // MIDDLE SERVO

    // clockwise looking from the right side
    right_i--;
    R_roll_servo.write(pitchAngles[right_i]); // physically is pitch but on mine is roll, BASE SERVO
    R_yaw_servo.write(yawAngles[right_i]); // MIDDLE SERVO

    float easingFactor = sin(left_i * PI / steps); // 0 at start/end, 1 at middle
    int delayTime = 10 + (int)(10 * (1 - easingFactor)); // 10–20 ms delay

    delay(delayTime);
  }
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
