#define PRESSURE_READ_PIN A1

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.print("Pressure: ");
  Serial.println(readPressure(PRESSURE_READ_PIN));
  delay(1000);
}

float readPressure(uint8_t _pin) {  
  // 0.5V = 0 psi = 102
  // 4.5V = 150 psi = 922
  // 0.5 - 4.5 = 0.183 psi per 1 step
  uint16_t val = analogRead(_pin);
  if (val <= 102) {
    return 0.0;
  } else if (val >= 922) {
    return 150.0;
  } else {
    return (val - 102) * 0.183;
  }
}
