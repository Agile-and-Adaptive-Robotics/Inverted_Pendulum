void setup() {
  Serial.begin(9600);
  while(!Serial);

  int a = 2;
  int b = 257;


  Serial.print(b & 0xFF);
  Serial.print((uint8_t) b);

  Serial.println();

  

}

void loop() {
  // put your main code here, to run repeatedly:

}
