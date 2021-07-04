#include "FlySkyIBus.h"
#include "FlySkyIBusSENS.h"

const uint8_t count_values = 10;
uint16_t ibus_values[count_values];
uint16_t previous_values[count_values];
FlySkyIBus ibus(Serial2, ibus_values, count_values);

extern uint16_t sense_callback(uint8_t cb_type, uint8_t cb_index);
FlySkyIBusSENS isens(Serial1, &sense_callback);
void setup() {
  pinMode(BOARD_LED_PIN, OUTPUT);
  // put your setup code here, to run once:
  while (!Serial && millis() < 5000);
  Serial.begin(115200);
  delay(250);
  isens.begin();
  ibus.begin();


}

void loop() {
  // put your main code here, to run repeatedly:
  isens.step();
  ibus.step(0);

  if (ibus.hasFreshFrame()) {
    bool value_changed = false;
    for (uint8_t i = 0; i < count_values; i++) {
      if (ibus_values[i] != previous_values[i]) value_changed = true;
      previous_values[i] = ibus_values[i];
    }
    if (value_changed) {
      for (uint8_t i = 0; i < count_values; i++) {
        Serial.printf("%5d ", ibus_values[i]);
      }
      Serial.println();
    }
  }
}

uint16_t cb_value = 0;
uint16_t sense_callback(uint8_t cb_type, uint8_t cb_index) {
  if (cb_index != 1) return 0;
  switch (cb_type) {
    case FLYSENS_DISCOVER:
      return 1;
    case FLYSENS_TYPE:
      return 3; // External voltage
    case FLYSENS_VALUE:
      cb_value += 10; // not sure of range yet.
      return cb_value;
  }
}
