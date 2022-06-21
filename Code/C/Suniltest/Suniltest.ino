float angle = PI / 2;
const float targetAngle = PI / 2;


// valve open amounts-- floats from 0 to 1-- 1 = fullly open, 0 = closed
float leftValveOpen = 0
float rightValveOpen = 0







// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);

}




// the loop function runs over and over again forever
void loop() {


  int randNum = random(100);

  Serial.println(randNum);
}
