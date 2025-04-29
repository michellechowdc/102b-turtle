#include <ESP32Servo.h>

static const int yawPin = 13;
static const int pitchPin = 27;
static const int rollPin = 12;

Servo yawServo;
Servo pitchServo;
Servo rollServo;

const int xPin = 33; 
const int yPin = 14; 
const int buttonPin = 32; 


const int deadZone = 100;  // joystick values within this range from middle will be ignored

// previous servo positions
int prevYaw = -1;
int prevPitch = -1;

// joystick reading
int xValue, yValue, xMovement, yMovement, yaw, pitch;

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

  timer = timerBegin(1000000);           // 1 MHz
  timerAttachInterrupt(timer, &timerISR); 
  timerAlarm(timer, 1000000, true, 0);
}

void setup() {
  Serial.begin(115200);

  yawServo.setPeriodHertz(50);  
  yawServo.attach(yawPin);

  pitchServo.setPeriodHertz(50);
  pitchServo.attach(pitchPin);

  rollServo.setPeriodHertz(50);
  rollServo.attach(rollPin);

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
      yawServo.write(yaw); 
      prevYaw = yaw;  // set prevyaw to previous yaw so if there is no update use that instead of resetting to 0
      Serial.print("Yaw (theta): ");
      Serial.println(yaw);
    }
  }

  if (yMovement == 1) {
    pitch = map(yValue, 0, 4095, 45, 180); // physical boundary at less than 45 deg
    
    if (abs(pitch - prevPitch) > 2) { 
      pitchServo.write(pitch); 
      prevPitch = pitch;     
      Serial.print("Pitch (phi): ");
      Serial.println(pitch);
    }
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

      yawServo.write(yaw);
      pitchServo.write(pitch);
      rollServo.write(roll);  

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
