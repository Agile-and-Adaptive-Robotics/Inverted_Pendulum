#define MISO 5
#define MOSI 6
#define SCK 3
#define SS 7

#define RESET 8

#define VCC 10
#define GND 12

unsigned int currentAddress = 0;

// programming codes


uint8_t programmingEnable[4] = {0xAC, 0x53, 0x00, 0x00};
uint8_t chipErase[4] = {0xAC, 0x80, 0x00, 0x00};


uint8_t data[128]; // 128 bytes of data




// ------------------ assebly to opcode program ------------------ //

uint16_t ldi(unsigned int registerNum, int value) { // returns opcode for loading value into selected register
  uint16_t opcode = 0xE; // put in load code
  opcode <<= 4;

  opcode |= (int)(value / 16); // put in first hex digit of value
  opcode <<= 4;

  opcode |= registerNum - 16; // put in register number
  opcode <<= 4;

  opcode |= (int)(value % 16); // put in second hex digit of value

  return opcode;
}

uint16_t out(int value, unsigned int registerNum) {
  uint16_t opcode = 0xB; // put in out code
  opcode <<= 4;

  unsigned int secondDigit = 0x8;
  secondDigit += ((int)(value / 16)) * 2 + (registerNum > 15);
  opcode |= secondDigit;
  opcode <<= 4;

  if (registerNum > 15) registerNum -= 16;
  opcode |= registerNum; // put in register number
  opcode <<= 4;

  opcode |= (int)(value % 16); // put in second hex digit of value

  return opcode;
}

uint16_t eor(unsigned int registerNum1, unsigned int registerNum2) { // returns opcode for eor two registers
  uint16_t opcode = 0x2;
  opcode <<= 4;

  if (registerNum1 < 16 && registerNum2 < 16) {
    opcode |= 0x4;
  } else if (registerNum1 > 15 && registerNum2 < 16) {
    opcode |= 0x5;
    registerNum1 -= 16;
  } else if (registerNum1 < 16 && registerNum2 > 15) {
    opcode |= 0x6;
    registerNum2 -= 16;
  } else {
    opcode |= 0x7;
    registerNum1 -= 16;
    registerNum2 -= 16;
  }

  opcode <<= 4;
  
  opcode |= registerNum1;
  opcode <<= 4;

  opcode |= registerNum2;

  return opcode;
}

uint16_t adc(unsigned int registerNum1, unsigned int registerNum2) { // returns opcode for adding register2 to register1
  uint16_t opcode = 0x1;
  opcode <<= 4;

  if (registerNum1 < 16 && registerNum2 < 16) {
    opcode |= 0xC;
  } else if (registerNum1 > 15 && registerNum2 < 16) {
    opcode |= 0xD;
    registerNum1 -= 16;
  } else if (registerNum1 < 16 && registerNum2 > 15) {
    opcode |= 0xE;
    registerNum2 -= 16;
  } else {
    opcode |= 0xF;
    registerNum1 -= 16;
    registerNum2 -= 16;
  }

  opcode <<= 4;
  
  opcode |= registerNum1;
  opcode <<= 4;

  opcode |= registerNum2;

  return opcode;
}

const uint16_t ret = 0x9508;


// --------------------------- debugging functions -------------------------------- //

void printHex(uint8_t num) {
  Serial.print("0x");
  int firstDigit = num / 16;
  if (firstDigit < 10) Serial.print(firstDigit);
  else Serial.print((char)(firstDigit + 55));

  int secondDigit = num % 16;

  if (secondDigit < 10) Serial.println(secondDigit);
  else Serial.println((char)(secondDigit + 55));
}

void printInBits(uint8_t b) {
  int toPrint = b;
  for (int i = 0; i < 8; i++) {
    Serial.print((b & 128) ? "1" : "0");
    b <<= 1;
  }
  Serial.print(": "); printHex(toPrint); Serial.println("");
}


// --------------------------- transfer functions ---------------------------------- //

uint8_t transferByte(uint8_t b) {
  int toPrint = b;
  uint8_t receivedByte = 0;
  
  Serial.print(">>>> ");
  
  for (int i = 0; i < 8; i++) {
    bool currentBit = b & 128;
    b <<= 1;

    // ASSUMING CPOL = 0 AND CPHA = 0 AND MSB IS SENT FIRST

    digitalWrite(MOSI, currentBit); // write MOSI to value of current bit to be read
    delay(10);

    digitalWrite(SCK, HIGH); // write clock to high to signal slave to read the MOSI value
    receivedByte <<= 1; // push the byte to the left to make room for the next incoming byte on the right end
    if (digitalRead(MISO)) receivedByte |= digitalRead(MISO); // add MISO signal to the end of the received byte
    Serial.print(currentBit);
    delay(10);
    
    digitalWrite(SCK, LOW); // finish clock cycle

  }
  Serial.print(": "); printHex(toPrint);
  Serial.print("<<<< "); printHex(receivedByte);

  return receivedByte;
}

void loadWordToPageBuffer(uint8_t highByte, uint8_t lowByte) {
  // ------- load high byte ------- // send code with address and data
  transferByte(0x48);
  transferByte(0x00);
  transferByte((uint8_t) (currentAddress)); // send address of LSB
  transferByte(highByte);
  
  currentAddress++; // new address for next byte
  
  delay(100); // wait for byte to be written
  
// ------------------------------------------------- //

  // ------- load low byte ------- // send code with address and data
  transferByte(0x40);                     // first byte of code
  transferByte(0x00);                     // second byte of code
  transferByte((uint8_t) (currentAddress - 1)); // send address
  transferByte(lowByte);                  // send byte to load

  currentAddress++; // new address for next byte

  delay(100); // wait for byte to be written
}

uint8_t writePageBufferToFlash(uint8_t pageStartAddress, uint8_t pageEndAddress) {
  transferByte(0x4C);             // instruction code (from datasheet)
  transferByte(pageStartAddress); // address of most important bit of page
  transferByte(pageEndAddress);   // address of least important bit of page
  transferByte(0x00);             // instruction code (from datasheet)
}





void pulseReset() {
  digitalWrite(RESET, HIGH);

  delay(20);

  digitalWrite(RESET, LOW);
}


void setup() {
  Serial.begin(19200);
  while(!Serial);

  Serial.println("");
  Serial.println("--------- Starting ---------");
  Serial.println("");

  // -------------- initialize arduino as programmer by setting pins ---------------------- //

  pinMode(VCC, OUTPUT);
  pinMode(GND, OUTPUT);


  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(SS, OUTPUT);

  pinMode(RESET, OUTPUT);

  digitalWrite(SS, HIGH); // disable signal for now

  digitalWrite(RESET, LOW);
  digitalWrite(SCK, LOW);

  delay(100);
  
  digitalWrite(VCC, HIGH);
  digitalWrite(GND, LOW);

  delay(100);

  pulseReset();


  delay(10);


  digitalWrite(SS, LOW);

  delay(20);
  Serial.println("---- programming enable ----");

  uint8_t echo = 0;
  int maxTries = 5;
  int tryAgainCounter = 0;
  while (echo != 0x53 && tryAgainCounter < maxTries) {
    tryAgainCounter++;

    if (tryAgainCounter > 1) Serial.println("---- out of sync, trying again ----");

    pulseReset();
    transferByte(programmingEnable[0]);
    transferByte(programmingEnable[1]);
    echo = transferByte(programmingEnable[2]);
    transferByte(programmingEnable[3]);
    
  }

  if (tryAgainCounter == maxTries) {
    Serial.println("---- failed to get in sync ----");
    while(1); // stop program (infinite loop)
  }


  delay(100);

  // -------------------- chip erase ---------------------- //
  
  Serial.println("---- chip erase ----");

  transferByte(chipErase[0]);
  transferByte(chipErase[1]);
  transferByte(chipErase[2]);
  transferByte(chipErase[3]);


  delay(100); // wait for memory to erase (recommended by atmel is 9.0ms)

  Serial.println("---- load in data to page buffer ----");

  // --------------------- load opcodes to page buffer in peripheral chip ------------------ //

  unsigned int startAddress = currentAddress;


  DDRB = 0x04;  // port b data register code
  PORTB = 0x05; // port b data direction register code
    
  unsigned int codeLength = 3;
  uint16_t code[3] = {
    ldi (16, (1 << 6)),    // load 01000000 to register 16 (for turning on 6th bit of DDRB and PORTB)
    out (DDRB, 16),        // write register 16 to DDRB
    out (PORTB, 16)        // write register 16 to PORTB
  };

  delay(20);

  for (int i = 0; i < codeLength; i++) {
    uint8_t highByte = code[i] >> 8;
    uint8_t lowByte = code[i] & 0xFF;

    loadWordToPageBuffer(highByte, lowByte);
  }

  unsigned int endAddress = currentAddress - 1;

  // -------------------------- write peripheral's page buffer to the program memory (flash) ------------------- //

  Serial.println("---- write page data to flash ----");
  
  writePageBufferToFlash(startAddress, endAddress);

  delay(100);

  // ------------------------- read flash data to see if loading and writing worked ------------------ //

  Serial.println("---- read flash data ----");

  for (int i = startAddress; i <= endAddress; i++) {
    transferByte(0x28);
    transferByte(i);
    transferByte(i + 1);
    transferByte(0x00);  
  }


  digitalWrite(SS, HIGH);

  digitalWrite(RESET, HIGH); // commence normal operation



}

void loop() {

}
