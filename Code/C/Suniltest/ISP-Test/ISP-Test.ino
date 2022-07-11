void setup() {
  Serial.begin(9600);
  while(!Serial);

  int a = 2;
  int b = 5;


  Serial.println();

  for (int i = 0; i < 10; i++) {
    Serial.print(a << i); Serial.print(", ");
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
