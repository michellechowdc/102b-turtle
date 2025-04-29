#define BIN_1 26    
#define BIN_2 25
#define LED_PIN 13
#define VRX_PIN 36
#define VRY_PIN 4
#define SW_PIN 14

//int MAX_PWM_VOLTAGE = 255;   // Max PWM duty cycle

void setup() {
  pinMode(BIN_1, OUTPUT);
  pinMode(BIN_2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SW_PIN, INPUT_PULLUP);

  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
}

void loop() {
  int xVal = analogRead(VRX_PIN);
  int yVal = analogRead(VRY_PIN);
  int swPressed = digitalRead(SW_PIN);
  int speed = map(yVal, 0, 4095, -255, 255);

  // Dead zone to prevent motor jitter at center
  if (abs(speed) < 30) {
    speed = 0;
  }

  if (speed > 0) {
    // Forward
    analogWrite(BIN_1, 0);
    analogWrite(BIN_2, speed);
    digitalWrite(LED_PIN, HIGH);
  } else if (speed < 0) {
    // Reverse
    analogWrite(BIN_1, -speed);  // Remove negative sign
    analogWrite(BIN_2, 0);
    digitalWrite(LED_PIN, LOW);
  } else {
    // Stop
    analogWrite(BIN_1, 0);
    analogWrite(BIN_2, 0);
    digitalWrite(LED_PIN, LOW);
  }
  delay(100);
}
