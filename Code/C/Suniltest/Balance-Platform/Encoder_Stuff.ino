


int getEncoderPosition() {
  static int pos = 0;
  encoder.tick();

  int newPos = analogRead(1);
  if (pos != newPos) {
    //Serial.print("pos:");
    //Serial.print(newPos);
    //Serial.print(" dir:");
    //Serial.print((int)(encoder.getDirection()));
    Serial.println();
    pos = newPos;
  }


  return newPos;
}
