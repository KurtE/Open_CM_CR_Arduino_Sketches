extern char end asm("end");
void setup() {
  // put your setup code here, to run once:
  while (!Serial) ;

  Serial.begin(115200);
  Serial.println("Malloc test");
  uint8_t *malloc_addrs[17];
  malloc_addrs[0] = (uint8_t *)&end;
  for (uint8_t i = 1; i < 17; i++) {
    malloc_addrs[i] = (uint8_t *)malloc(i);
    Serial.printf("size:%d, addr:%x delta:%x\n", i, (uint32_t)malloc_addrs[i],
                  (uint32_t)(malloc_addrs[i] - malloc_addrs[i - 1]));
  }
  Serial.println();
  // write4bytesTxRx allocates 16 bytes. Lets allocate a few of them to see how
  // Yes I am leaving memory...
  for (uint8_t i = 1; i < 17; i++) {
    malloc_addrs[i] = (uint8_t *)malloc(16);
    Serial.printf("size: 16, addr:%x delta:%x\n", (uint32_t)malloc_addrs[i],
                  (uint32_t)(malloc_addrs[i] - malloc_addrs[i - 1]));
  }

  // Now see what happens if we try to realloc them...
  Serial.println("\nTry realloc and see when they move...");
  for (uint8_t i = 1; i < 17; i++) {
    uint8_t* new_addr = (uint8_t*)realloc(malloc_addrs[i], 16 + i);
    if (new_addr == malloc_addrs[i]) {
      Serial.printf("realloc: addr:%x size: %d same address\n", (uint32_t)malloc_addrs[i], 16+i);
    } else {
      Serial.printf("realloc: addr:%x size: %d *** new address %x ***\n", (uint32_t)malloc_addrs[i], 16+i, 
          (uint32_t)new_addr);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
