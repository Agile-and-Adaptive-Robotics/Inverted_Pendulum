
uint8_t myArray[5][5] = {
  {1, 2, 3, 4, 5},
  {1, 2, 3, 4, 5},
  {1, 2, 3, 4, 5},
  {1, 2, 3, 4, 5},
  {1, 2, 3, 4, 5}
  
};



uint16_t ldi(int registerNum, int value) { // returns opcode for loading value into selected register
  uint16_t opcode = 0xE; // put in load code
  opcode <<= 4;

  opcode |= (int)(value / 16); // put in first hex digit of value
  opcode <<= 4;

  opcode |= registerNum - 16; // put in register number
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


uint16_t out(int value, unsigned int registerNum) {
  uint16_t opcode = 0xB; // put in out code
  opcode <<= 4;

  unsigned int secondDigit = 0x8;
  secondDigit += ((int)(value / 16) * 2) + (registerNum > 15);
  opcode |= secondDigit;
  opcode <<= 4;

  if (registerNum > 15) registerNum -= 16;
  opcode |= registerNum; // put in register number
  opcode <<= 4;

  opcode |= (int)(value % 16); // put in second hex digit of value

  return opcode;
}




void setup() {
  Serial.begin(19200);
  while(!Serial);

  int a = 0b10;
  int b = 257;



  

}

void loop() {
  // put your main code here, to run repeatedly:

}
