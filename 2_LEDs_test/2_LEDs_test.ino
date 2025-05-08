///////////////// LED code
const int redLEDPin = 33;
const int yelLEDPin = 15;

void setup() {
  // put your setup code here, to run once:
  pinMode(redLEDPin, OUTPUT);
  pinMode(yelLEDPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(redLEDPin, HIGH);  // turn the LED on (HIGH is the voltage level)
  digitalWrite(yelLEDPin, LOW);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(redLEDPin, LOW);   // turn the LED off by making the voltage LOW
  digitalWrite(yelLEDPin, HIGH);   // turn the LED off by making the voltage LOW
  delay(1000);    
}
