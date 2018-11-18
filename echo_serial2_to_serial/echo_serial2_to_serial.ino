uint32_t time_last_input = 0;
void setup() {
  while (!Serial) ; // wait for serial port to be available.
  Serial.begin(115200);
  Serial2.begin(57600);
}

void loop() {
  if (Serial2.available()) {
    while(Serial2.available()) {
      uint8_t ch = Serial2.read();
      Serial.printf("%02x ", ch);
    }
    time_last_input = millis();
  }
  if (time_last_input && ((millis() - time_last_input) > 50)) {
    Serial.println(); // break to new line
    time_last_input = 0;
  }
}
