uint8_t input_line[20];
uint8_t input_count = 0;
uint32_t last_packet;

void setup() {
  Serial.begin(115200);
  Serial2.begin(57600);
  pinMode(BOARD_LED_PIN, OUTPUT);
  last_packet = millis();
}

void loop() {
  if (Serial.available()) {
    Serial2.print((char)Serial.read());//send data coming from USB to Serial2
  }

  if (Serial2.available()) {
    input_count = 0;
    uint32_t delta_time_between_packets = millis() - last_packet;
    last_packet = millis();
    uint32_t time_first_byte = last_packet;
    uint32_t time_last_input = last_packet;

    while (((millis() - time_last_input) < 15) && (input_count < 6)) {
      if (Serial2.available()) {
        input_line[input_count++] = Serial2.read();
        toggleLED();
        time_last_input = millis();
      }
    }
    uint32_t delta_time_packet = millis() - time_first_byte;
    Serial.print(delta_time_between_packets, DEC);
    Serial.print(" ");
    Serial.print(delta_time_packet, DEC);
    Serial.print(": ");
    for (uint8_t i = 0; i < input_count; i++) {
      Serial.print(input_line[i], HEX);
      Serial.print(" ");
    }
    Serial.print(": ");
    for (uint8_t i = 0; i < input_count; i++) {
      if ((input_line[i] >= ' ') && (input_line[i] <= '|'))
        Serial.print((char)input_line[i]);
      else
        Serial.print(".");
    }
    Serial.println();
  }
}
