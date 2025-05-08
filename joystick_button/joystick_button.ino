const int buttonPin = 27; 
volatile bool buttonIsPressed = false; 

int state = 1;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // 50 milliseconds

void IRAM_ATTR btn_isr() {  // the function to be called when interrupt is triggered
  unsigned long currentTime = millis();
  // Check if enough time has passed since the last press
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    buttonIsPressed = true;
    lastDebounceTime = currentTime;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(buttonPin, INPUT);  // configures the specified pin to behave either as an input or an output
  attachInterrupt(buttonPin, btn_isr, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  switch (state) {
    case 1: // button not pressed
      if (event_ButtonPressed()) {
        state = 2;
      } else {
        Serial.println("Button not pressed");
      }
      break;

    case 2: // button pressed
      if (event_ButtonPressed()) {
        state = 1;
      } else {
        Serial.println("Button Pressed");
      }
      break;
  }
}

bool event_ButtonPressed() {
  if (buttonIsPressed == true) {
    buttonIsPressed = false;
    return true;
  } else {
    return false;
  }
}
