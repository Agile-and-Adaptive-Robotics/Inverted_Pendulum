/* This code uses time proportioning PWM control on the bang-bang valve manifolds.
 *  
 * pin mapping reference: https://www.arduino.cc/en/Hacking/PinMapping2560
 *  
 * digital pin 22 = transistor array P1 = PA0/AD0 = pin78
 * digital pin 23 = transistor array P2 = PA1/AD1 = pin77
 */

// libraries___________________________________________________________________________________
/*encoder code library: https://github.com/mathertel/RotaryEncoder
 */

#include <Adafruit_FXAS21002C.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <RotaryEncoder.h>

// Assign devices______________________________________________________________________________
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
RotaryEncoder encoder(2, 3);

// constants___________________________________________________________________________________
const int thresholdPressure = 1; // arbitrary value

// variables___________________________________________________________________________________
float pressureSensor1 = 0;
float pressureSensor2 = 0;

// functions___________________________________________________________________________________

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

void setUpTimerInterrupt(void){ // should work but register values unchecked
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

void setPinModes(void){
  // set pins as outputs; same as pinMode
  
  // Register A
  DDRA = B00000011; // data direction register (i/o)
  PORTA = 0; // port A data register (high/low)
}

void bangHigh(int pin){
  // use bitwise operators here to not alter set pin modes
  PORTA |= (1 << (pin - 22));
}

void bangLow(int pin){
  PORTA &= ~(1 << (pin - 22));
}

void bangBang(float desiredPressure, float actualPressure, int pin){
  float lowerPressure;
  float upperPressure;

  // compute upper and lower bounds.
  lowerPressure = desiredPressure - thresholdPressure;
  upperPressure = desiredPressure + thresholdPressure;

  // open or close valve.
  if (actualPressure > upperPressure){
    bangLow(pin);
  }
  else if (actualPressure < lowerPressure){
    bangHigh(pin);
  }
}

void manualPWM(float dutyCycle, int pin){ // if dutyCycle is 50% then input is 0.5
  float period = 2; // seconds
  float timeHigh = period * dutyCycle; // seconds
  float timeLow = period - (period * dutyCycle); // seconds

  bangHigh(pin);
  delay(timeHigh);
  bangLow(pin);
  delay(timeLow);
}

//_____________________________________________________________________________________________

void setup() {
  setPinModes();
  setUpTimerInterrupt();
  
  Serial.begin(9600);

  while (!Serial) {
    delay(1); //delay until serial starts up
  }
  
  displaySensorDetails();
}

float timerInterrupt(){
  // read analog pressure sensors
  pressureSensor1 = analogRead(A2);
  pressureSensor2 = analogRead(A3);

  return pressureSensor1, pressureSensor2;
}

ISR(TIMER4_COMPA_vect){
  // timer interrupt execute commands
  // generates pulse wave of freq 1Hz/2 = 0.5Hz (two cycles for full wave)

  timerInterrupt();
}

void loop() {
  // put your main code here, to run repeatedly:

}
