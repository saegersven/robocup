// "Voltmeter" using voltage divider in combination with a red Led, to indicate battery voltage
int led_pin = 4;
void setup() {
  pinMode(led_pin, OUTPUT);
}

void loop() {
  if (analogRead(A3) < 541) { //6,5V
    digitalWrite(led_pin, HIGH);
  }
  else if (analogRead(A3) < 583) { //7V
    blink(100);
  }
  else if (analogRead(A3) < 625) { // 7,5V
    blink(500);
  }
  else {
    digitalWrite(led_pin, LOW);
  }
  
}

void blink(int delay_ms) {
  for (int i = 0; i < 3; i++) {
    digitalWrite(led_pin, HIGH);
    delay(delay_ms);
    digitalWrite(led_pin, LOW);
    delay(delay_ms);
  }
}
