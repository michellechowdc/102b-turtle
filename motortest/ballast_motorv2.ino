#include <driver/ledc.h>

#define BIN_1 26    
#define BIN_2 25
#define LED_PIN 13
#define VRX_PIN 36
#define VRY_PIN 4
#define SW_PIN 14

enum MotorState {
  IDLE,
  FORWARD,
  REVERSE
};

MotorState currentState = IDLE;
volatile bool checkJoystick = false; // Event flag set by Timer interrupt

// Constants
const int DEAD_ZONE = 30;
hw_timer_t *timer = NULL;
volatile bool interruptCounter = false; 
int totalInterrupts = 0; 
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


void IRAM_ATTR timerISR() {
 portENTER_CRITICAL_ISR(&timerMux);
  checkJoystick = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}
void TimerInterruptInit() {

  timer = timerBegin(10000);           // 1 MHz
  timerAttachInterrupt(timer, &timerISR); 
  timerAlarm(timer, 10000, true, 0);

  //Serial.println("System ready");
}

void setup() {
  pinMode(BIN_1, OUTPUT);
  pinMode(BIN_2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SW_PIN, INPUT_PULLUP);

  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  TimerInterruptInit();
}


void loop() {
  if (checkJoystick) {
    checkJoystick = false;  // Clear the flag

    int yVal = analogRead(VRY_PIN);
    int speed = map(yVal, 0, 4095, -255, 255);

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
    case FORWARD:
      analogWrite(BIN_1, 0);
      analogWrite(BIN_2, speed);
      digitalWrite(LED_PIN, HIGH);
      Serial.println("Moving Forward");
      break;

    case REVERSE:
      analogWrite(BIN_1, -speed);
      analogWrite(BIN_2, 0);
      digitalWrite(LED_PIN, LOW);
      Serial.println("Moving Reverse");
      break;

    case IDLE:
      analogWrite(BIN_1, 0);
      analogWrite(BIN_2, 0);
      digitalWrite(LED_PIN, LOW);
      Serial.println("Motor Stopped");
      break;
  }
}
