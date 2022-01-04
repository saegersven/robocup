// "Voltmeter" using voltage divider in combination with a red Led, to indicate battery voltage
int led_pin = 4;
void setup() {
  pinMode(led_pin, OUTPUT);
}
//67,567

//473 für 7V

void loop() {  
  if (analogRead(A3) < 405) { // Voltage < 6V
    digitalWrite(led_pin, HIGH);
  }
  else if (analogRead(A3) < 439) { // Voltage < 6.5V
    blink(100);
  }
  else if (analogRead(A3) < 473) { // Voltage < 7V
    blink(500);
  } else { // Voltage > 7V
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
