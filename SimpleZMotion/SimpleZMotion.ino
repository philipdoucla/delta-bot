const int stepPinX = 11;
const int dirPinX = 12;
const int stepPinY = 9;
const int dirPinY = 10;
const int stepPinZ = 8;
const int dirPinZ = 7;

void setup() {
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
}

void loop() {
  digitalWrite(dirPinX, HIGH);
  digitalWrite(dirPinY, HIGH);
  digitalWrite(dirPinZ, HIGH);

  for(int i = 0; i < 200; i++) {
    digitalWrite(stepPinX, HIGH);
    digitalWrite(stepPinY, HIGH);
    digitalWrite(stepPinZ, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinX, LOW);
    digitalWrite(stepPinY, LOW);
    digitalWrite(stepPinZ, LOW);
    delayMicroseconds(500);
  }
  delay(10000);

  digitalWrite(dirPinX, LOW);
  digitalWrite(dirPinY, LOW);
  digitalWrite(dirPinZ, LOW);

  for(int i = 0; i < 100; i++) {
    digitalWrite(stepPinX, HIGH);
    digitalWrite(stepPinY, HIGH);
    digitalWrite(stepPinZ, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinX, LOW);
    digitalWrite(stepPinY, LOW);
    digitalWrite(stepPinZ, LOW);
    delayMicroseconds(500);
  }
  delay(1000);
}
