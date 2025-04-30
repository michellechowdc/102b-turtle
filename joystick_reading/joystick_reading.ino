const int xPin = 12;  // Joystick X-axis connected to analog pin A0
const int yPin = 15;  // Joystick Y-axis connected to analog pin A1

void setup() {
  Serial.begin(115200);  // Start the serial communication
}

void loop() {
  int xVal = analogRead(xPin);  // Read the X-axis
  int yVal = analogRead(yPin);  // Read the Y-axis

  Serial.print("X: ");
  Serial.print(xVal);
  Serial.print("  Y: ");
  Serial.println(yVal);

  if (yVal < 3110) {
    Serial.println("forward");

    if (xVal > 3250) {
      Serial.println("right");
    }

    if (xVal < 3230) {
      Serial.println("left");
    }

    // otherwise back is in neutral position
  }

  delay(100);  // Delay for readability
}
