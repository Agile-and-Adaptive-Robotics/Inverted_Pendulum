// plan
  // design controller to figure out which commands to output
  // valve manifold only on/off (bang-bang)
    // PWM
  // Cascaded control
  // timer interrupt not void loop to read pressure sensors & IMU
  // IMU & pressure sensors & encoder & SD card slot
  // hopefully figure out OOP

// MEGA uses timer4, CTC
// timer interrupt instructable: https://www.instructables.com/id/Arduino-Timer-Interrupts/
// https://forum.arduino.cc/index.php?topic=625904.0

// bitwise operators (| & <<): https://www.programiz.com/c-programming/bitwise-operators

#include <Adafruit_FXAS21002C.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Assign devices
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

//vars
int intFreq = 0

// functions
void displaySensorDetails(void) {
  sensor_t sensor;
  gyro.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    0x");
  Serial.println(sensor.sensor_id, HEX);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" rad/s");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" rad/s");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" rad/s");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

int interruptFreq() {
  intFreq = clockSpeed / (prescaler * (compareMatchRegister + 1))
}

void setUpTimerInterrupt(void){
  cli();

  // set timer4 interrupt at 1Hz
  TCCR4A = 0; //register set to 0
  TCCR4B = 0; //register set to 0
  TCNT4 = 0; //init counter to 0

  // set compare match register for 1Hz increments
  OCR4A = 15624/1;

  // turn on CTC mode
  TCCR4B |= (1 << WGM12);

  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR4B |= (1 << CS12) | (1 << CS10);

  // enable timer compare interrupt
  TIMSK4 |= (1 << OCIE4A);

  sei();
}

void setup() {
  setUpTimerInterrupt();
  
  Serial.begin(9600);

  while (!Serial) {
    delay(1); //delay until serial starts up
  }
  
  displaySensorDetails();
}

ISR(TIMER4_COMPA_vect){
  // timer interrupt execute commands
  // generates pulse wave of freq 1Hz/2 = 0.5Hz (two cycles for full wave)
}

void loop() {
  // put your main code here, to run repeatedly:

}

void timerInterrupt(){
  // pause loop sequence; constant sampling freq
}
