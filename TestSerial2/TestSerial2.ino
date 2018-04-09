char buffer[80];
void setup() {
  // put your setup code here, to run once:
  while (!Serial && (millis() < 3000));
  pinMode(0, OUTPUT);
  Serial2.begin(38400);
  Serial.begin(115200);
  Serial2.setTimeout(100);

  Serial.println("Test XBee code");
  delay(10);
}

uint8_t read_bytes_until(char term, char *pb, uint8_t cb) {
  uint32_t start_time = millis();
  uint8_t count = 0;
  while ((count < cb) && ((millis()-start_time) < 100)) {
    if (Serial2.available()) {
      char ch = Serial2.read();
      if (ch == term) break;
      *pb++ = ch;
      count++;
    }
  }
  return count;
}

void loop() {
   // Test XBEE 
  memset(buffer, sizeof(buffer), 0); 
  Serial2.print("+++");
  Serial2.flush();  // make sure it goes out

  uint32_t start_time = millis();
  digitalWrite(0, HIGH);
  //uint8_t len = Serial2.readBytesUntil('\r', buffer, sizeof(buffer)); 
  uint8_t len = read_bytes_until('\r', buffer, sizeof(buffer)); 
  digitalWrite(0, LOW);
  uint32_t dt = millis() - start_time;
  Serial.print("Return len: ");
  Serial.print(len, DEC);
  Serial.print(" DT: ");
  Serial.println(dt, DEC);
  if (len) Serial.write(buffer, len);

  Serial.print("Inspect buffer: ");
  for (int i=0; i < 10; i++) {
    if (buffer[i] == 0) break;
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" Avail: ");
  uint8_t avail = Serial2.available();
  Serial.print(avail);
  Serial.print(": ");
  while (avail--) {
    Serial.print(Serial2.read(), HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  Serial2.println("ATCN");  // exit command mode if we get there...
  Serial.println();
  delay(250);
}
