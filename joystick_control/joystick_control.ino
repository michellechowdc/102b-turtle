#define JoystickX 
#define JoystickY
int xValue = 0; 
int yValue = 0;

void setup() {
  serial.begin(115200)

}

void loop() {
  // put your main code here, to run repeatedly:
  xValue = analogRead(JoystickX);
  yValue = analogRead(JOystickY);
Serial.print("X = ")
Serial.print(xValue)
Serial.print(", Y = ")
Serial.print(YValue)
delay(100);
}
