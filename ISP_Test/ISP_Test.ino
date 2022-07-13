
#include <SPI.h>

#define BAUDRATE 19200


// normal spi pins (10, 11, 12, 13) were acting weird

#define MISO 5
#define MOSI 6
#define SCK 3
#define SS 7

#define VCC 8
#define GND 7

#define WREN  6
#define WRDI  4
#define RDSR  5
#define WRSR  1
#define READ  3
#define WRITE 2

const unsigned int numPages = 256;
const unsigned int pageSize = 64;

unsigned int currentAddress = 0;

// programming codes

uint8_t programmingEnable[4] = {0xAC, 0x53, 0x00, 0x00};
uint8_t chipErase[4] = {0xAC, 0x80, 0x00, 0x00};


byte clr;
uint8_t buff[128]; // holds data in the data buffer (128 bytes of data)



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


char transferChar(uint8_t data) {

  SPDR = data; // write byte to SPI Data Register to begin transfer

  while(!(SPSR & (1 << SPIF))); // wait for SPI Interrupt Flag to be set to 1
  
  /* explanation:
   *  this waits while (SPSR & (1 << SPIF)) == 00000000
   *  SPSR represents the SPI Status Register
   *  SPIF represents the position of the SPI Interrupt Flag in the SPSR
   *  (SPSR & (1 << SPIF)) returns 1 if both are 1, and 0 if either is 0
   *  if the SPIF bit in the SPSR matches (1 << SPIF) then that means that the SPIF bit is 1
   *  this returns a 1 because the SPIF bits are both 1 and so the while boolean is is false
   */

  

  return SPDR;

}




uint8_t manualTransferByte(uint8_t b) {
  int toPrint = b;
  uint8_t receivedByte = 0;
  
  int waitTime = 1000000000; // idk really what this value should be, better wait longer to be safe?

  Serial.print("sending: ");
  
  for (int i = 0; i < 8; i++) {
    bool currentBit = b & 128;
    b <<= 1;

    // ASSUMING CPOL = 0 AND CPHA = 0 AND MSB IS SENT FIRST

    digitalWrite(MOSI, currentBit); // write MOSI to value of current bit to be read

    //delayMicroseconds(waitTime); // wait for signal to be set
    delay(100);

    digitalWrite(SCK, HIGH); // write clock to high to signal slave to read the MOSI value

    Serial.print(currentBit);
    
    receivedByte <<= 1; // push the byte to the left to make room for the next incoming byte on the right end
    if (digitalRead(MISO)) receivedByte |= digitalRead(MISO); // add MISO signal to the end of the received byte

    //delayMicroseconds(waitTime); // wait for slave to read the signal
    delay(100);
    
    digitalWrite(SCK, LOW); // finish clock cycle

    
    

  }
  Serial.print(": "); printHex(toPrint);
  printHex(receivedByte);

  return receivedByte;
}


uint8_t transfer4Bytes(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  manualTransferByte(a);
  manualTransferByte(b);
  manualTransferByte(c);
  return manualTransferByte(d);
}


void loadWordToPageBuffer(uint8_t lowByte, uint8_t highByte) {
  // ------- load low byte ------- // send code with address and data
  manualTransferByte(0x40);                     // first byte of code
  manualTransferByte(0x00);                     // second byte of code
  manualTransferByte((uint8_t) currentAddress); // send address
  manualTransferByte(lowByte);                  // send byte to load

  currentAddress++; // new address for next byte

  delay(100); // wait for byte to be written
// ------------------------------------------------- //
  
  // ------- load high byte ------- // send code with address and data
  manualTransferByte(0x48);
  manualTransferByte(0x00);
  manualTransferByte((uint8_t) (currentAddress - 1)); // send address of LSB
  manualTransferByte(highByte);
  
  currentAddress++; // new address for next byte
  
  delay(100); // wait for byte to be written
}

uint8_t writePageBufferToFlash(uint8_t pageStartAddress, uint8_t pageEndAddress) {
  manualTransferByte(0x4C);             // instruction code (from datasheet)
  manualTransferByte(pageStartAddress); // address of most important bit of page
  manualTransferByte(pageEndAddress);   // address of least important bit of page
  manualTransferByte(0x00);             // instruction code (from datasheet)
}


void setup() {
  Serial.begin(BAUDRATE);
  while(!Serial);

  Serial.println("");
  Serial.println("--------- Starting ---------");
  Serial.println("");


  SPI.begin();


  /* SPCR
| 7    | 6    | 5    | 4    | 3    | 2    | 1    | 0    |
| SPIE | SPE  | DORD | MSTR | CPOL | CPHA | SPR1 | SPR0 |

SPIE - Enables the SPI interrupt when 1

SPE - Enables the SPI when 1

DORD - Sends data least Significant Bit First when 1, most Significant Bit first when 0

MSTR - Sets the Arduino in controller mode when 1, peripheral mode when 0

CPOL - Sets the data clock to be idle when high if set to 1, idle when low if set to 0

CPHA - Samples data on the falling edge of the data clock when 1, rising edge when 0

SPR1 and SPR0 - Sets the SPI speed, 00 is fastest (4MHz) 11 is slowest (250KHz)
*/


  // . pull . down slave select pin
  // after . each data packet, slave select is pulled high to synchronize

  // SPCR = SPI control register
  // register is a byte of microcontroller memory that can be reaf from or written to
  // used for: control, data, and status


  pinMode(A2, OUTPUT);
  pinMode(A4, OUTPUT);


  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(SS, OUTPUT);

  digitalWrite(SS, HIGH); // disable signal for now

  
  digitalWrite(A2, HIGH);
  digitalWrite(A4, LOW);


  // SPI control register settings:
  //SPCR = (1 << SPE) | (1 << MSTR); // SPE and MSTR are the number of spots from the right the SPE and MSTR settings are in the SPCR

  // SPSR = SPI status register
  // SPDR = SPI data register


  // MSB = most significant bit
  // LSB = least significant bit

  // most significant means has the most affect on the number
  //.. leftmost bit is the most significant because it has the most affect on the number
  

  // clear SPSR and SPDR

  //clr = SPSR;
  //clr = SPDR;

  delay(10);

/* Instructions from Atmel datasheet:
1. Power-up sequence:
Apply power between VCC and GND while RESET and SCK are set to “0”. In some systems, the programmer can
not guarantee that SCK is held low during power-up. In this case, RESET must be given a positive pulse of at
least two CPU clock cycles duration after SCK has been set to “0”.
2. Wait for at least 20ms and enable serial programming by sending the programming enable serial instruction to pin
MOSI.
3. The serial programming instructions will not work if the communication is out of synchronization. When in sync.
the second byte (0x53), will echo back when issuing the third byte of the programming enable instruction. Whether
the echo is correct or not, all four bytes of the instruction must be transmitted. If the 0x53 did not echo back, give
RESET a positive pulse and issue a new programming enable command.
4. The flash is programmed one page at a time. The memory page is loaded one byte at a time by supplying the 6
LSB of the address and data together with the load program memory page instruction. To ensure correct loading
of the page, the data low byte must be loaded before data high byte is applied for a given address. The program
memory page is stored by loading the write program memory page instruction with the 7 MSB of the address. If
polling (RDY/BSY) is not used, the user must wait at least tWD_FLASH before issuing the next page (see
Table 27-16). Accessing the serial programming interface before the flash write operation completes can result in
incorrect programming.
5. A: The EEPROM array is programmed one byte at a time by supplying the address and data together with the
appropriate Write instruction. An EEPROM memory location is first automatically erased before new data is
written. If polling (RDY/BSY) is not used, the user must wait at least tWD_EEPROM before issuing the next byte (see
Table 27-16). In a chip erased device, no 0xFFs in the data file(s) need to be programmed.
B: The EEPROM array is programmed one page at a time. The memory page is loaded one byte at a time by
supplying the 6 LSB of the address and data together with the Load EEPROM memory page instruction. The
EEPROM memory page is stored by loading the write EEPROM memory page instruction with the 7 MSB of the
address. When using EEPROM page access only byte locations loaded with the Load EEPROM memory page
instruction is altered. The remaining locations remain unchanged.
If polling (RDY/BSY) is not used, the used must wait at least tWD_EEPROM before issuing the next byte (see
Table 27-16). In a chip erased device, no 0xFF in the data file(s) need to be programmed.
6. Any memory location can be verified by using the read instruction which returns the content at the selected
address at serial output MISO.
7. At the end of the programming session, RESET can be set high to commence normal operation.
8. Power-off sequence (if needed):
Set RESET to “1”.
Turn VCC power off.
Table 27-16. Typical Wait Delay Before Writing the Next
 */

  digitalWrite(SS, LOW);

  delay(20);
  Serial.println("---- programming enable ----");
  
  manualTransferByte(programmingEnable[0]);
  manualTransferByte(programmingEnable[1]);
  manualTransferByte(programmingEnable[2]);
  manualTransferByte(programmingEnable[3]);


  delay(100);
  Serial.println("---- chip erase ----");

  // erase program memory and EEPROM ***** THIS WORKED I THINK !!!!! ***** (blink program stopped running after I executed the program with this)

  //manualTransferByte(chipErase[0]);
  //manualTransferByte(chipErase[1]);
  //manualTransferByte(chipErase[2]);
  //manualTransferByte(chipErase[3]);


  delay(20); // wait for memory to erase (recommended by atmel is 9.0ms)

  Serial.println("---- load in data to page buffer ----");

  // transfer data

  unsigned int startAddress = currentAddress;

  for (int i = 0; i < 64; i += 2) {
    //loadWordToPageBuffer(i, i + 1);
  }

  unsigned int endAddress = currentAddress - 1;

  Serial.println("---- write page data to flash ----");

  
  
  //writePageBufferToFlash(startAddress, endAddress);

  


/*
  for (int i = 0; i < 128; i++) {
    uint8_t receivedByte = manualTransferByte(127 - i);
    Serial.print("sending: "); printInBits(127 - i);
    Serial.print("receive: "); printInBits(receivedByte);
  }
*/
  digitalWrite(SS, HIGH);



}

void loop() {

}
