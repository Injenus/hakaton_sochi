byte joy[4] = {14, 15, 16, 17};

void setup() {
  Serial.begin(9600);
  for (byte i = 2; i < 5; i++) {
    pinMode(i, INPUT_PULLUP);
  }
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
}

void loop() {
  //  for (byte i = 0; i++; i < 4) {
  //    Serial.print(analogRead(joy[i]));
  //    Serial.print(',');
  //  }
  Serial.print(analogRead(14));
  Serial.print(',');
  Serial.print(analogRead(15));
  Serial.print(',');
  Serial.print(analogRead(16));
  Serial.print(',');
  Serial.print(analogRead(17));
  Serial.print(',');
  Serial.print(digitalRead(2));
  Serial.print(',');
  Serial.print(digitalRead(4));
  Serial.println();



  delay(10);
}
