#include <DynamixelSDK.h>
#define DXL_DIR 28
#define DXL_TXD 29
#define DXL_RXD 30
void setup() {
  while (!Serial && (millis() < 3000)) ;  // Give time for Teensy and other USB arduinos to create serial port
  Serial.begin(38400);  // start off the serial port.
  Serial.println("\nCM9.04 Servo TTL level test program");
  pinMode(DXL_DIR, OUTPUT);
  for (int i=0;i<4; i++) {
    digitalWrite(DXL_DIR, !digitalRead(DXL_DIR));
    delay(5);
  }
  digitalWrite(DXL_DIR, LOW);
  
  pinMode(DXL_TXD, OUTPUT);
  digitalWrite(DXL_TXD, LOW);
  pinMode(DXL_RXD, INPUT);
  
  pinMode(BOARD_LED_PIN, OUTPUT);
  for (int i=0;i<5; i++) {
    digitalWrite(BOARD_LED_PIN, !digitalRead(BOARD_LED_PIN));
    delay(250);
  }
  delay(250);
  pinMode(BOARD_LED_PIN, INPUT);
}

void loop() {
}

