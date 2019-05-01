int controlInf = 2;
int controlVac = 3;

void setup() {
  pinMode(controlInf, OUTPUT);
  pinMode(controlVac, OUTPUT);

  digitalWrite(controlInf, HIGH); // HIGH = off!
  digitalWrite(controlVac, HIGH);

  Serial.begin(9600);
}

void loop() {
  digitalWrite(controlInf, LOW);
  digitalWrite(controlVac, HIGH); // begin to inflate
  delay(1000);
  digitalWrite(controlInf, HIGH);
  digitalWrite(controlVac, LOW); // begin to vacuum
  delay(5000);
  digitalWrite(controlInf, HIGH);
  digitalWrite(controlVac, HIGH); // stop both!
  delay(2500);
}
