
#define CS   10
#define MOSI 11
#define MISO 12
#define SCK  13




void setup() {
  Serial.begin(19200);

  pinMode(CS,   INPUT );
  pinMode(MOSI, INPUT );
  pinMode(MISO, OUTPUT);
  pinMode(SCK,  INPUT );

  

}

void printBits(uint8_t b) {
  for (int i = 0; i < 8; i++) {
    bool currentBit = 0x80 & b;
    Serial.print(currentBit);
    b <<= 1;
  }
}

void loop() {
  printBits(SPDR); Serial.println();

  delay(1000);

}
