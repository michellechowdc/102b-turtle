#include <driver/ledc.h>

#define BIN_1 26    
#define BIN_2 25
#define LED_PIN 13
#define VRX_PIN 36
#define VRY_PIN 4
#define SW_PIN 14

//the 3 motor states dependent on joystick
enum MotorState {
  IDLE, //joystick in neutral
  FORWARD, //up position, CW, SINK
  REVERSE //down position, CCW, FLOAT
};

MotorState currentState = IDLE;
volatile bool checkJoystick = false; // Event flag set by Timer interrupt

//establish the interrupt and timer variables
const int DEAD_ZONE = 30; //for the joystick sensitivity
hw_timer_t *timer = NULL;
volatile bool interruptCounter = false; 
int totalInterrupts = 0; 
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//interrupt service routine triggered by the hardware timer
void IRAM_ATTR timerISR() {
 portENTER_CRITICAL_ISR(&timerMux);
  checkJoystick = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}
//initiallizing the timer
void TimerInterruptInit() {

  timer = timerBegin(10000);          
  timerAttachInterrupt(timer, &timerISR); 
  timerAlarm(timer, 10000, true, 0);

  //Serial.println("System ready");
}
// begin setting up the pins by initializing
void setup() {
  pinMode(BIN_1, OUTPUT);
  pinMode(BIN_2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SW_PIN, INPUT_PULLUP);

  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  TimerInterruptInit(); // very important to start the timer
}

//event driven loop
void loop() {
  if (checkJoystick) {
    checkJoystick = false;  // Clear the flag

    int yVal = analogRead(VRY_PIN);
    int speed = map(yVal, 0, 4095, -255, 255);

// this section is so the motor doesn't move it if is accidentally moved a bit out of neutral position on accident

    if (abs(speed) < DEAD_ZONE) {
      speed = 0;
    }

    MotorState newState = determineState(speed);

    if (newState != currentState) {
      currentState = newState;
      handleState(currentState, speed);
    }
  }

}

MotorState determineState(int speed) {
  if (speed > 0) {
    return FORWARD;
  } else if (speed < 0) {
    return REVERSE;
  } else {
    return IDLE;
  }
}

void handleState(MotorState state, int speed) {
  switch (state) {
    case FORWARD: //switch case, foward, CW, SINK
      analogWrite(BIN_1, 0);
      analogWrite(BIN_2, speed);
      digitalWrite(LED_PIN, HIGH);
      //Serial.println("Moving CW");
      break;

    case REVERSE: //switch case, reverse, CCW, FLOAT
      analogWrite(BIN_1, -speed);
      analogWrite(BIN_2, 0);
      digitalWrite(LED_PIN, LOW);
      //Serial.println("Moving CCW");
      break;

    case IDLE: // joystick in neutral, motor speed = 0
      analogWrite(BIN_1, 0);
      analogWrite(BIN_2, 0);
      digitalWrite(LED_PIN, LOW);
      //Serial.println("Motor Stopped");
      break;
  }
}
