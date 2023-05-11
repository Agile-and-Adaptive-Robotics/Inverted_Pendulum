/*






Send programming capability to next MCU with array containing opcodes for program as well as programming capability
  - every microcontroller has some kind of pin input signal to know where on the string of MCUs it is

If next MCU has a matching signal, it writes the program opcodes to the next MCU's program memory
OR, if next MCU has a matching signal, it writes the program opcodes to it's own memory

PROGRAMMING EXECUTION:

- MCU receives through incoming SPI port: programming capability, array with program opcodes, array with programming capability opcodes, and pin identification value:
  - It executes programming program (which was written to program memory)
    - this first sets the identification pin to input and reads the pin:
      - if the pin input matches the pin identification value then it self programs the program opcodes to it's flash memory (atmel devices have this functionality)
      - if the pin input DOES NOT match the pin identification value, it sends the same package it received to the next MCU through it's outward SPI port



ALTERNATIVE TO PIN IDENTIFICATION VALUE:

- Each MCU passes on an integer value that decreases by one each time it is passed down and it 
  reacts if value == 0




PROGRAMMING PACKAGE INCLUDES:

- Program data written to memory (series of 16-bit opcodes)
- Counter integer value written to memory
- Programming capability code that is written straight to program memory:
  - Subtracts 1 from counter integer
  - If counter integer == 0:
    - Self write program data to program memory
  - If counter != 0:
    - Send programming package to next MCU
  




 
*/
